#!/usr/bin/env python3
import socket
import struct
import numpy as np
import rospy
import threading
from sensor_msgs.msg import JointState

# This script should be run on another device, which acts as the Piper host, and interact with scripts/cpp/rrtc_example.cc

# ─── Config ───────────────────────────────────────────────────────────────────
BOARD_IP = "192.168.50.106"
UDP_PORT = 9876
TCP_HOST = "0.0.0.0"
TCP_PORT = 8080

MAGIC = b"VAMP_PC\x00"  # 8 bytes


def recv_exact(conn: socket.socket, n: int) -> bytes:
    buf = bytearray()

    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            break

        buf.extend(chunk)

    return bytes(buf)


def recv_path(conn: socket.socket) -> bytes:
    """
    TCP Protocol:
    SEQ_ID      uint32
    N_WAYPOINTS uint32
    WAYPOINTS   N * 6 * float32
    """
    header = recv_exact(conn, 8)
    if len(header) < 8:
        return b""

    seq_id, n_wp = struct.unpack_from("<II", header, 0)
    payload_size = n_wp * 6 * 4
    payload = recv_exact(conn, payload_size)

    return header + payload


def pack_udp(seq_id: int, joint_state: np.ndarray) -> bytes:
    """
    UDP Protocol:
    MAGIC       8 bytes
    SEQ_ID      uint32
    JOINT_STATE 6 * float32
    """
    seq_blob = struct.pack("<I", seq_id)
    js_blob = joint_state.astype(np.float32).tobytes()

    return MAGIC + seq_blob + js_blob


def unpack_tcp(data: bytes):
    """
    TCP Protocol:
    SEQ_ID      uint32
    N_WAYPOINTS uint32
    WAYPOINTS   N * 6 * float32
    """
    if len(data) < 8:
        raise ValueError("TCP payload too short")

    seq_id, n_wp = struct.unpack_from("<II", data, 0)
    expected = 8 + n_wp * 6 * 4

    if len(data) < expected:
        raise ValueError(f"TCP payload truncated: got {len(data)}, expected {expected}")

    waypoints = []
    offset = 8

    for _ in range(n_wp):
        wp = struct.unpack_from("<6f", data, offset)
        waypoints.append(np.array(wp, dtype=np.float32))
        offset += 24

    return seq_id, waypoints


def make_tcp_server() -> socket.socket:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((TCP_HOST, TCP_PORT))
    srv.listen(1)
    print(f"[TCP] Server listening on {TCP_HOST}:{TCP_PORT}")
    return srv


class PiperController:
    def __init__(self):
        rospy.init_node("my_controller")

        self.pub = rospy.Publisher("joint_states", JointState, queue_size=2)

        self.trajectory_lock = threading.Lock()
        self.current_waypoints = []
        self.current_wp_index = 0

        self.seq_id = 0
        self.latest_seq_sent = 0
        self.latest_seq_accepted = 0
        self.seq_send_time = {}
        self.last_request_time = 0.0

        # Adjustable
        self.control_rate = 50.0
        self.request_rate = 10.0
        self.max_response_age = 0.5

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate),
            self.control_loop
        )

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.tcp_srv = make_tcp_server()
        self.tcp_srv.settimeout(0.02)

    def next_seq_id(self) -> int:
        self.seq_id = (self.seq_id + 1) & 0xFFFFFFFF

        if self.seq_id == 0:
            self.seq_id = 1

        return self.seq_id

    def control_loop(self, event):
        wp_to_publish = None

        with self.trajectory_lock:
            if self.current_waypoints and self.current_wp_index < len(self.current_waypoints):
                wp_to_publish = self.current_waypoints[self.current_wp_index]
                self.current_wp_index += 1

        if wp_to_publish is None:
            return

        cmd = JointState()
        cmd.header.stamp = rospy.Time.now()
        cmd.position = wp_to_publish.tolist() + [0.0]
        cmd.velocity = [0, 0, 0, 0, 0, 0, 20]
        cmd.effort = [0, 0, 0, 0, 0, 0, 1]

        self.pub.publish(cmd)

    def read_joint_state(self):
        try:
            joint_msg = rospy.wait_for_message(
                "joint_states_single",
                JointState,
                timeout=0.05
            )
        except rospy.ROSException:
            return None

        joint_state = np.array(joint_msg.position, dtype=np.float32)

        if joint_state.shape[0] >= 7:
            joint_state = np.delete(joint_state, 6)

        if joint_state.shape[0] != 6:
            print(f"[ROS] Bad joint_state length: {joint_state.shape[0]}")
            return None

        return joint_state

    def maybe_send_request(self):
        now = rospy.Time.now().to_sec()

        if now - self.last_request_time < 1.0 / self.request_rate:
            return

        joint_state = self.read_joint_state()
        if joint_state is None:
            return

        seq = self.next_seq_id()
        self.latest_seq_sent = seq
        self.seq_send_time[seq] = now
        self.last_request_time = now

        pkt = pack_udp(seq, joint_state)
        self.udp_sock.sendto(pkt, (BOARD_IP, UDP_PORT))

        print(f"[UDP] Sent seq={seq}")

        # Clean old seq record
        old_keys = [
            key for key, sent_time in self.seq_send_time.items()
            if now - sent_time > 5.0
        ]

        for key in old_keys:
            del self.seq_send_time[key]

    def maybe_receive_response(self):
        try:
            conn, addr = self.tcp_srv.accept()
        except socket.timeout:
            return

        with conn:
            raw = recv_path(conn)

        if not raw:
            return

        try:
            resp_seq, waypoints = unpack_tcp(raw)
        except ValueError as e:
            print(f"[TCP] Parse error: {e}")
            return

        now = rospy.Time.now().to_sec()
        sent_time = self.seq_send_time.get(resp_seq)

        print(
            f"[TCP] Received seq={resp_seq}, "
            f"accepted={self.latest_seq_accepted}, "
            f"N={len(waypoints)}"
        )

        if resp_seq <= self.latest_seq_accepted:
            print(
                f"[TCP] Drop old path: "
                f"resp_seq={resp_seq}, accepted={self.latest_seq_accepted}"
            )
            return

        if sent_time is None:
            print(f"[TCP] Drop unknown seq: resp_seq={resp_seq}")
            return

        age = now - sent_time
        if age > self.max_response_age:
            print(
                f"[TCP] Drop expired path: "
                f"resp_seq={resp_seq}, age={age:.3f}s"
            )
            return

        self.latest_seq_accepted = resp_seq

        if len(waypoints) >= 2:
            with self.trajectory_lock:
                self.current_waypoints = waypoints
                self.current_wp_index = 1

            print(f"[Preempted] New path loaded with {len(waypoints)} waypoints.")
        else:
            with self.trajectory_lock:
                self.current_waypoints = []
                self.current_wp_index = 0

            print(f"[TCP] Empty path for seq={resp_seq}, holding.")

    def run(self):
        rate = rospy.Rate(100)

        try:
            while not rospy.is_shutdown():
                self.maybe_send_request()
                self.maybe_receive_response()
                rate.sleep()

        except rospy.ROSInterruptException:
            print("\n[ROS] Interrupted")
        except KeyboardInterrupt:
            print("\n[Main] Interrupted")
        finally:
            self.udp_sock.close()
            self.tcp_srv.close()


if __name__ == "__main__":
    controller = PiperController()
    controller.run()
