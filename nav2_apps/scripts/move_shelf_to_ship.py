import time
from copy import deepcopy
from attach_shelf.srv import GoToLoading
from geometry_msgs.msg import PoseStamped, PolygonStamped, Polygon, Point32
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class ServiceClient(Node):

    def __init__(self):
        # Constructor de la clase. Este método se llama cuando se crea una nueva instancia de la clase.
        super().__init__('service_client')  # Inicialización de la superclase (Node).

        # Creación del cliente de servicio y conexión a 'approach_shelf'.
        self.client = self.create_client(GoToLoading, 'approach_shelf')

        # Variables de estado inicial.
        self.start_client = False
        self.recieve_f_approach = False
        self.service_done = False

        # Creación del temporizador que llama a timer_callback cada segundo.
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Este método se llama cada vez que el temporizador se activa.
        if self.start_client:
            time.sleep(3)
            self.timer.cancel()
            # Espera a que el servicio esté disponible, revisando cada 4 segundos.
            while not self.client.wait_for_service(timeout_sec=4.0):
                self.get_logger().info("Service Unavailable. Waiting for Service...")
  
            # Creación de la solicitud y establecimiento de la variable attach_to_shelf.
            request = GoToLoading.Request()
            request.attach_to_shelf = self.recieve_f_approach
            self.service_done = False

            # Envío de la solicitud de manera asíncrona.
            self.future = self.client.call_async(request)
            # Cuando se reciba la respuesta, se llamará a response_callback.
            self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        # Este método se llama cuando se recibe una respuesta del servicio.
        try:
            response = future.result()
            self.service_done = True
            self.get_logger().info("[Result] SUCCESS!")
        except Exception as e:
            self.get_logger().info("Service In-Progress...")
            self.service_done = False

    def is_service_done(self):
        # Método que devuelve el estado de la solicitud de servicio.
        return self.service_done
#ServiceClient Class

class NavigationClass(Node):

    def __init__(self):
        super().__init__('navigation_node')
        self.change_footprint = False
        self.restart_footprint = False
        self.navigator = BasicNavigator()

        self.initial_pose = PoseStamped()
        self.set_loading_position = PoseStamped()
        self.set_shipping_position = PoseStamped()
        self.global_publisher_ = self.create_publisher(PolygonStamped, '/global_costmap/published_footprint', 10)
        self.local_publisher_ = self.create_publisher(PolygonStamped, '/local_costmap/published_footprint', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Creamos el objeto de clase ServiceClient
        self.approach_to_object_client = ServiceClient()

        # asume que hay funciones para definir los valores de la pose
        self.set_init_pose()
        self.navigator.waitUntilNav2Active()
        self.set_loading_pose()
        #self.start_Checkpoint_5()
        # self.set_shipping_pose()
        print("START CHECKPOINT5")
    def timer_callback(self):
        # Iniciar el cambio de footprint cuando cargamos el objeto
        if self.change_footprint:
            msg = PolygonStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'robot_odom'  # replace with the appropriate frame_id if needed

            # Define the vertices of the rectangular footprint (e.g., a 2x2 square around the origin)
            footprint = Polygon()
            footprint.points = [Point32(x, y, 0.0) for x, y in [(-0.30, -0.30), (-0.30, 0.30), (0.30, 0.30), (0.30, -0.30)]]
            msg.polygon = footprint
            self.global_publisher_.publish(msg)
            self.local_publisher_.publish(msg)
            self.get_logger().info('Publishing footprint + object')

        # Resetear el footprint cuando dejemos el objeto
        if self.restart_footprint:
            msg = PolygonStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'robot_odom'  # replace with the appropriate frame_id if needed

            # Define the vertices of the rectangular footprint (e.g., a 2x2 square around the origin)
            footprint = Polygon()
            footprint.points = [Point32(x, y, 0.0) for x, y in [(-0.15, -0.15), (-0.15, 0.15), (0.15, 0.15), (0.15, -0.15)]]
            msg.polygon = footprint
            self.global_publisher_.publish(msg)
            self.local_publisher_.publish(msg)
            self.get_logger().info('Publishing footprint')

    def set_init_pose(self):
        # establece el valor de la pose inicial
        # Set your demo's initial pose
        #real
        self.initial_pose.pose.position.x = 1.5433
        self.initial_pose.pose.position.y = 1.7673
        self.initial_pose.pose.orientation.x = 0.0
        self.initial_pose.pose.orientation.y = 0.0
        self.initial_pose.pose.orientation.z = -0.070032
        self.initial_pose.pose.orientation.w = -1.0
        self.navigator.setInitialPose(self.initial_pose)

        #sim 
        # self.initial_pose.pose.position.x = 0.400
        # self.initial_pose.pose.position.y = 1.400
        # self.initial_pose.pose.orientation.x = 0.0
        # self.initial_pose.pose.orientation.y = 0.0
        # self.initial_pose.pose.orientation.z = 0.0
        # self.initial_pose.pose.orientation.w = 1.0
        # self.navigator.setInitialPose(self.initial_pose)
        pass

    def set_loading_pose(self):
        # establece el valor de la pose de carga
        #real
        self.set_loading_position.header.frame_id = 'map'
        self.set_loading_position.header.stamp = self.navigator.get_clock().now().to_msg()
        self.set_loading_position.pose.position.x =  4.71866
        self.set_loading_position.pose.position.y = 4.7842
        self.set_loading_position.pose.orientation.x = 0.0
        self.set_loading_position.pose.orientation.y = -0.0
        self.set_loading_position.pose.orientation.z = 0.452171
        self.set_loading_position.pose.orientation.w = -0.89193
        self.navigator.goToPose(self.set_loading_position)

        #Sim
        # self.set_loading_position.pose.position.x = 5.212375359967021
        # self.set_loading_position.pose.position.y = -1.685197975067205
        # self.set_loading_position.pose.orientation.x = -3.7369382824396817e-07
        # self.set_loading_position.pose.orientation.y = 4.9089843952456745e-06
        # self.set_loading_position.pose.orientation.z = -0.7115463283701283
        # self.set_loading_position.pose.orientation.w = 0.7026391837627273
        # self.navigator.goToPose(self.set_loading_position)
            
        i = 0
        while not self.navigator.isNavComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print("FEEDBACK")

        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print("RESULT")
        elif result == NavigationResult.CANCELED:
            print("CANCELED")
        elif result == NavigationResult.FAILED:
            print("FAILED")
            exit(-1)
        while not self.navigator.isNavComplete():    
            pass
        pass

    def start_Checkpoint_5(self):
        # Hacer que se posicione debajo del objeto
        self.approach_to_object_client.recieve_f_approach = True
        self.approach_to_object_client.start_client = True
        print("Starting approaching to object")
        pass

    def set_shipping_pose(self):

        #real
        self.set_shipping_position.header.frame_id = 'map'
        self.set_shipping_position.header.stamp = self.navigator.get_clock().now().to_msg()
        self.set_shipping_position.pose.position.x = 3.4948867147127434
        self.set_shipping_position.pose.position.y = 0.07018892895451764
        self.set_shipping_position.pose.orientation.x = 0.0
        self.set_shipping_position.pose.orientation.y = -0.0
        self.set_shipping_position.pose.orientation.z =  0.36744791909367003
        self.set_shipping_position.pose.orientation.w = -0.9300440993596657
        self.navigator.goToPose(self.set_shipping_position)
        pass

        #sim
        # self.set_shipping_position.header.frame_id = 'map'
        # self.set_shipping_position.header.stamp = self.navigator.get_clock().now().to_msg()
        # self.set_shipping_position.pose.position.x = -0.5080764926403905
        # self.set_shipping_position.pose.position.y = -0.07634239137160069
        # self.set_shipping_position.pose.orientation.x = 0.0
        # self.set_shipping_position.pose.orientation.y = -0.0
        # self.set_shipping_position.pose.orientation.z = -0.9304336018897295
        # self.set_shipping_position.pose.orientation.w = -0.36646051966685916
        # self.navigator.goToPose(self.set_shipping_position)
        # pass
    
    def startNavigation(self):
        #ABER
        pass

def main(args=None):

    rclpy.init(args=args)
    nav_to_pose_client = NavigationClass()
    approach_client = nav_to_pose_client.approach_to_object_client

    executor = MultiThreadedExecutor()
    executor.add_node(nav_to_pose_client)
    executor.add_node(approach_client)

    try:
        nav_to_pose_client.get_logger().info('Beginning client, shut down with CTRL-C')
        approach_client.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()

    except KeyboardInterrupt:
        nav_to_pose_client.get_logger().info('Keyboard interrupt, shutting down.\n')
    nav_to_pose_client.destroy_node()
    approach_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
