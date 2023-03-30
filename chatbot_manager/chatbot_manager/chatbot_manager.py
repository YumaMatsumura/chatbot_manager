import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from whisper_msgs.srv import CreateTranscript
from chat_gpt_msgs.srv import AskChatGpt
from voicevox_msgs.srv import Speak

class ChatbotManager(Node):
    def __init__(self):
        super().__init__('chatbot_manager')
        self.cli_whisper = self.create_client(CreateTranscript, 'create_transcript')
        self.cli_chatgpt = self.create_client(AskChatGpt, 'ask_chat_gpt')
        self.cli_voicevox = self.create_client(Speak, 'speak')
        
        while not self.cli_whisper.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('whisper_ros service not available, waiting again...')
        while not self.cli_chatgpt.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('chatgpt_ros service not available, waiting again...')
        while not self.cli_voicevox.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('voicevox_ros service not available, waiting again...')
        
        
        self.request_to_create_transcript()
        
    def request_to_create_transcript(self):
        self.req_create_transcript = CreateTranscript.Request()
        self.future_create_transcript = self.cli_whisper.call_async(self.req_create_transcript)
        self.future_create_transcript.add_done_callback(self.handle_create_transcript_response)
    
    def request_to_ask_chatgpt(self, text):
        self.req_ask_chatgpt = AskChatGpt.Request()
        self.req_ask_chatgpt.request_message = text
        self.future_ask_chatgpt = self.cli_chatgpt.call_async(self.req_ask_chatgpt)
        self.future_ask_chatgpt.add_done_callback(self.handle_ask_chatgpt_response)
    
    def request_to_speak(self, text):
        self.req_speak = Speak.Request()
        self.req_speak.text = text
        self.future_speak = self.cli_voicevox.call_async(self.req_speak)
        self.future_speak.add_done_callback(self.handle_speak_response)
        
    def handle_create_transcript_response(self, future):
        self.future_create_transcript = None
        try:
            response = future.result()
            if not response.result:
                raise Exception('whisper_ros has failed.')
            self.get_logger().info('whisper_ros succeeded.')
            self.request_to_ask_chatgpt(response.transcript)
        except Exception as e:
            self.get_logger().error('whisper_ros service call failed %r' & (e,))
            self.request_to_create_transcript()
    
    def handle_ask_chatgpt_response(self, future):
        self.future_ask_chatgpt = None
        try:
            response = future.result()
            if not response.result:
                raise Exception('chat_gpt_ros has failed.')
            self.get_logger().info('chat_gpt_ros succeeded.')
            self.request_to_speak(response.response_message)
        except Exception as e:
            self.get_logger().error('chat_gpt_ros service call failed %r' & (e,))
            self.request_to_create_transcript()
    
    def handle_speak_response(self, future):
        self.future_speak = None
        try:
            response = future.result()
            if not response.result:
                raise Exception('voicevox_ros has failed.')
            self.get_logger().info('voicevox_ros succeeded.')
            self.request_to_create_transcript()
        except Exception as e:
            self.get_logger().error('voicevox_ros service call failed %r' & (e,))
            self.request_to_create_transcript()
    
        
def main(args=None):
    rclpy.init(args=args)
    
    try:
        chatbot_manager = ChatbotManager()
        executor = SingleThreadedExecutor()
        executor.add_node(chatbot_manager)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            chatbot_manager.destroy_node()
    finally:
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
