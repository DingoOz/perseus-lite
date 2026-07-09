"""
Companion to tensor_rt_launch.py: publishes a zero-filled input TensorList
shaped for mobilenetv2-1.0 (NCHW [1,3,224,224], float32) on `tensor_pub`,
and verifies a TensorList comes back on `tensor_sub` with mobilenetv2-1.0's
known classification-output shape ([1,1000], float32, name "output") --
the same properties NVIDIA's own isaac_ros_tensor_rt_test.py checks, just
without pulling in isaac_ros_test/IsaacROSBaseTest. First run also has to
wait out real TensorRT engine-build time (parsing the ONNX graph, engine
optimization) before the node is ready to infer, hence the long timeout.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from isaac_ros_tensor_list_interfaces.msg import Tensor, TensorList, TensorShape

EXPECTED_NAME = 'output'
EXPECTED_DATA_TYPE = 9  # float32
EXPECTED_DIMS = [1, 1000]
EXPECTED_RANK = 2
ENGINE_BUILD_TIMEOUT_SEC = 400


class TensorRTCheck(Node):
    def __init__(self):
        super().__init__('tensor_rt_check')
        self.received = []
        self.sub = self.create_subscription(
            TensorList, 'tensor_sub', self.on_msg, 10)
        self.pub = self.create_publisher(TensorList, 'tensor_pub', 10)

    def on_msg(self, msg):
        self.received.append(msg)

    def make_input(self):
        shape = TensorShape(rank=4, dims=[1, 3, 224, 224])
        tensor = Tensor(
            name='input',
            shape=shape,
            data_type=EXPECTED_DATA_TYPE,
            strides=[],
            data=[0] * (150528 * 4),
        )
        return TensorList(tensors=[tensor])


def main():
    rclpy.init()
    node = TensorRTCheck()
    input_msg = node.make_input()

    end_time = time.time() + ENGINE_BUILD_TIMEOUT_SEC
    sent = 0
    while time.time() < end_time and not node.received:
        sent += 1
        node.pub.publish(input_msg)
        rclpy.spin_once(node, timeout_sec=0.2)

    ok = bool(node.received)
    if ok:
        tensor = node.received[-1].tensors[0]
        checks = {
            'name': (tensor.name, EXPECTED_NAME),
            'data_type': (tensor.data_type, EXPECTED_DATA_TYPE),
            'rank': (tensor.shape.rank, EXPECTED_RANK),
            'dims': (list(tensor.shape.dims), EXPECTED_DIMS),
        }
        for key, (actual, expected) in checks.items():
            if actual != expected:
                ok = False
            node.get_logger().info(f'{key}: got={actual} expected={expected}')

    node.get_logger().info(f'sent={sent} received={len(node.received)}')
    print('TENSOR_RT OK' if ok else 'TENSOR_RT FAILED')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
