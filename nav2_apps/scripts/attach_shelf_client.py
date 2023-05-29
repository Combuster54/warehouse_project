from attach_shelf.srv import GoToLoading
import rclpy
from rclpy.node import Node
import time

class ServiceClient(Node):

    def __init__(self):
        # Constructor de la clase. Este método se llama cuando se crea una nueva instancia de la clase.
        super().__init__('service_client')  # Inicialización de la superclase (Node).
        
        # Creación del cliente de servicio y conexión a 'approach_shelf'.
        self.client = self.create_client(GoToLoading, 'approach_shelf')
        
        # Variables de estado inicial.
        self.start_client = False
        self.recieve_f_approach = True
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

def main(args=None):
    rclpy.init(args=args)
    service_client = ServiceClient()
    rclpy.spin(service_client)

    # Destrucción del nodo y cierre de ROS una vez que se termina de ejecutar el programa.
    service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
