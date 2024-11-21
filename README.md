# Instrucciones para añadir los mensajes ROS2 personalizados a MATLAB

Para añadir mensajes ROS2 personalizados presentes en el submódulo tcp_ros_packages a MATLAB, sigue estos pasos:

1. **Generar los archivos de mensajes ROS2 en MATLAB:**
  - Usa el comando `ros2genmsg` para generar los archivos de mensajes. Para ello, introduce el path absoluto hasta el paquete ros. Por ejemplo:
    ```matlab
    folderPath = '~/car_control/external/tcp_ros_packages';
    ros2genmsg(folderPath);
    ```

2. **Añadir los mensajes generados al entorno de MATLAB:**
  - Después de generar los archivos de mensajes, añade la carpeta generada al path de MATLAB usando el comando `addpath`. Por ejemplo:
    ```matlab
    addpath('~/car_control/external/tcp_ros_packages/matlab_msg_gen');');
    ```

3. **Verificar la instalación de los mensajes:**
  - Para asegurarte de que los mensajes se han añadido correctamente, puedes listar los tipos de mensajes disponibles usando el comando `ros2 msg list`. Por ejemplo:
    ```matlab
    ros2 msg list
    ```

Recuerda reemplazar `~/` con la ruta real al directorio donde tengas descargado car_control.