# Topic: /test topic
#
# Publisher usage:
#   from std_msgs.msg import String
#   self.publisher = self.create_publisher(String, '/test topic', 10)
#   msg = String()
#   msg.data = 'hello'
#   self.publisher.publish(msg)
#
# Subscriber usage:
#   from std_msgs.msg import String
#   self.subscription = self.create_subscription(
#       String, '/test topic', self.listener_callback, 10)
#
#   def listener_callback(self, msg):
#       self.get_logger().info(f'Received: {msg.data}')
