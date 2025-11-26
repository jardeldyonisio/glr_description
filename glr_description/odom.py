#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # Criar um cliente para o serviço get_entity_state
        self.client = self.create_client(GetEntityState, '/gazebo/get_entity_state')

        # Criar um publicador no tópico /odom
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Configurar temporizador para chamar a função periodicamente
        self.timer = self.create_timer(0.01, self.timer_callback)  # Publica a cada 0.1 segundos (10 Hz)

        # Inicializar a requisição ao serviço
        self.request = GetEntityState.Request()
        self.request.name = 'freedomvehicle'  # Nome da entidade para obter o estado (substitua aqui)

        # Esperar até que o serviço esteja disponível
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /get_entity_state...')
        print("aqui foi")

    def timer_callback(self):
        print('aq agora')

        # Enviar requisição para obter o estado da entidade
        future = self.client.call_async(self.request)
        print('hdusahuidsahudsuadhdis')
        rclpy.spin_until_future_complete(self, future)        
        print('aq agora dhsuadhasihdashui')

        if future.result() is not None:
            result = future.result()
            print(f"pose: {result.state.pose}")
            print(f"rdasdsa: {result.state.twist}")

            state = future.result().state

            # Criar mensagem Odometry com os dados obtidos do serviço
            odom_msg = Odometry()

            # Preencher os dados da pose
            odom_msg.pose.pose = state.pose

            # Preencher os dados da velocidade
            odom_msg.twist.twist = state.twist

            # Publicar a mensagem no tópico /odom
            self.odom_publisher.publish(odom_msg)

            self.get_logger().info(f'Publicado odometria: posição ({state.pose.position.x}, {state.pose.position.y})')
        else:
            self.get_logger().error('Falha ao chamar o serviço /get_entity_state')
        


def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdomPublisher()

    rclpy.spin(odom_publisher)

    rclpy.shutdown()

if __name__ == '__main__':
    main()