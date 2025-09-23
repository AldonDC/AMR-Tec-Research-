
**Estancia de Investigación - Algoritmos de Fusión Sensorial para Localización Vehicular**

![Sensor Fusion Overview](docs/images/sensor_fusion_banner.png)
*[Diagrama conceptual de la fusión RTK-GPS + LiDAR]*

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![Velodyne](https://img.shields.io/badge/LiDAR-Velodyne_VLP16-blue.svg)](https://velodynelidar.com/products/puck/)
[![GPS](https://img.shields.io/badge/GPS-RTK_Enabled-green.svg)]()
[![License](https://img.shields.io/badge/License-Academic-lightgrey.svg)]()

## 📋 Descripción del Proyecto

Esta estancia de investigación se enfoca en el desarrollo e implementación de algoritmos avanzados de fusión sensorial que combinan datos de **RTK-GPS** y **LiDAR Velodyne VLP-16** para obtener estimaciones precisas y robustas de la pose vehicular en tiempo real.

### Objetivos de la Investigación

**Objetivo Principal:**
Desarrollar un algoritmo de fusión sensorial que integre mediciones RTK-GPS y datos LiDAR para determinar la pose 6DOF (posición y orientación) de un vehículo con precisión centimétrica y robustez ante oclusiones de señal GPS.

**Objetivos Específicos:**
- Implementar algoritmos de odometría LiDAR usando ICP y NDT
- Desarrollar filtros de Kalman extendidos para fusión RTK-GPS/LiDAR
- Evaluar técnicas de SLAM con restricciones GPS
- Optimizar algoritmos para procesamiento en tiempo real
- Validar precisión mediante datos de campo y simulación

## 🧭 Marco Teórico

### Sensores Utilizados

#### RTK-GPS (Real-Time Kinematic)
![RTK System](docs/images/rtk_system.png)
*[Configuración del sistema RTK-GPS]*

**Características:**
- **Precisión**: ±2cm horizontal, ±5cm vertical
- **Frecuencia**: 10-20 Hz
- **Ventajas**: Referencia absoluta, precisión alta en exterior
- **Limitaciones**: Pérdida de señal en interiores/túneles, multitrayectoria

#### Velodyne VLP-16 LiDAR
![VLP-16 Specs](docs/images/vlp16_specs.png)
*[Especificaciones técnicas del VLP-16]*

**Especificaciones:**
- **Canales**: 16 láseres
- **Rango**: 100m
- **Precisión**: ±3cm
- **Frecuencia**: 5-20 Hz (300,000-600,000 puntos/s)
- **Campo de visión**: 360° horizontal, ±15° vertical

### Algoritmos de Fusión Sensorial

#### 1. Odometría LiDAR
```matlab
% Implementación ICP (Iterative Closest Point)
function T = icp_odometry(scan_current, scan_previous)
    % Registro punto a punto entre escaneos consecutivos
    [T, ~, rmse] = pcregistericp(scan_current, scan_previous, ...
        'Metric', 'pointToPoint', 'MaxIterations', 100);
end
```

#### 2. Filtro de Kalman Extendido (EKF)
```matlab
% Modelo de estado: [x, y, z, roll, pitch, yaw, vx, vy, vz]
function [x_pred, P_pred] = ekf_predict(x, P, u, Q, dt)
    % Predicción basada en modelo de movimiento
    F = jacobian_motion_model(x, u, dt);
    x_pred = motion_model(x, u, dt);
    P_pred = F * P * F' + Q;
end
```

#### 3. Fusión Multi-Modal
![Fusion Architecture](docs/images/fusion_architecture.png)
*[Arquitectura del algoritmo de fusión]*

**Pipeline de procesamiento:**
1. **Preprocesamiento**: Filtrado y sincronización temporal
2. **Odometría LiDAR**: Estimación de movimiento relativo
3. **Corrección GPS**: Actualización con mediciones absolutas
4. **Fusión EKF**: Combinación óptima de estimaciones
5. **Post-procesamiento**: Suavizado y validación

## 🔬 Metodología de Investigación

### Fase 1: Implementación de Algoritmos Base

**Odometría LiDAR:**
- [ ] Implementar ICP básico
- [ ] Desarrollar algoritmo NDT (Normal Distribution Transform)
- [ ] Comparar rendimiento ICP vs NDT
- [ ] Optimizar para tiempo real

**Integración RTK-GPS:**
- [ ] Parser de mensajes NMEA/RTCM
- [ ] Transformaciones de coordenadas (WGS84 ↔ UTM ↔ Local)
- [ ] Detección de pérdida de señal
- [ ] Modelo de incertidumbre adaptativo

### Fase 2: Desarrollo del Algoritmo de Fusión

**Filtro de Kalman Extendido:**
```matlab
% Estructura del estado del vehículo
state = struct(...
    'position', [x; y; z], ...           % Posición 3D
    'orientation', [roll; pitch; yaw], ... % Orientación
    'velocity', [vx; vy; vz], ...        % Velocidad lineal
    'angular_vel', [wx; wy; wz]);        % Velocidad angular
```

**Modelos de medición:**
- **GPS**: H_gps = [I_3x3, 0_3x6] (observa solo posición)
- **LiDAR**: Odometría relativa entre frames
- **IMU**: Orientación y aceleraciones (opcional)

### Fase 3: Validación y Optimización

**Métricas de evaluación:**
- Error de posición RMS
- Error de orientación RMS  
- Consistencia estadística (NEES/NIS)
- Tiempo de procesamiento

**Datasets de prueba:**
- Trayectorias urbanas con oclusiones GPS
- Entornos estructurados (estacionamientos)
- Escenarios dinámicos con obstáculos móviles

## Metodología de Fusión Sensorial RTK-GPS + LiDAR

### Enfoque de Combinación de Datos

El algoritmo desarrollado utiliza un enfoque probabilístico que combina las fortalezas complementarias de ambos sensores:

**RTK-GPS: Referencia Absoluta Global**
- Proporciona posición global precisa (±2cm) cuando hay línea de vista a satélites
- Actúa como ancla para prevenir deriva acumulativa
- Frecuencia de actualización: 10-20 Hz
- Vulnerable a oclusiones en entornos urbanos/interiores

**Velodyne VLP-16: Percepción Local Robusta**
- Genera mapas 3D detallados del entorno inmediato
- Proporciona odometría relativa mediante registro de nubes de puntos
- Robusto ante condiciones meteorológicas y pérdidas de señal satelital
- Procesamiento intensivo: requiere optimización algoritmica

### Estrategia de Fusión Implementada

El sistema combina ambas fuentes de información utilizando un Filtro de Kalman Extendido (EKF) que modela el estado completo del vehículo:

```matlab
% Estado del vehículo: pose 6DOF + velocidades
state_vector = [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]';

% Modelo de predicción basado en cinemática vehicular
function x_next = predict_state(x_current, control_input, dt)
    % Integración de velocidades para obtener nueva pose
    x_next = x_current + state_derivative(x_current, control_input) * dt;
end

% Corrección con mediciones GPS (cuando disponibles)
function update_with_gps(measurement_gps, noise_gps)
    H_gps = [eye(3), zeros(3,9)]; % Observa solo posición xyz
    innovation = measurement_gps - H_gps * state_estimate;
    kalman_gain = covariance * H_gps' / (H_gps * covariance * H_gps' + noise_gps);
    % Actualización del estado y covarianza
end

% Corrección con odometría LiDAR (siempre disponible)
function update_with_lidar(relative_transform, noise_lidar)
    % Convertir transformación relativa a innovación de estado
    predicted_motion = compute_predicted_motion(state_estimate, dt);
    innovation = transform_difference(relative_transform, predicted_motion);
    % Aplicar actualización EKF
end
```

### Manejo de Pérdidas de Señal GPS

Uno de los aspectos críticos del algoritmo es mantener precisión durante interrupciones de señal GPS:

1. **Detección de pérdida**: Monitoreo de calidad de señal y timeout de mensajes
2. **Modo degradado**: Confianza únicamente en odometría LiDAR con propagación de incertidumbre
3. **Re-adquisición**: Validación y fusión gradual al recuperar señal GPS
4. **Drift compensation**: Uso de landmarks LiDAR para reducir deriva acumulativa

## 🔧 Configuración del Entorno

### Dependencias MATLAB

**Toolboxes requeridos:**
```matlab
% Verificar toolboxes instalados
ver('lidar')          % Lidar Toolbox
ver('gps')            % GPS Toolbox (si disponible)
ver('robotics')       % Robotics System Toolbox
ver('nav')            % Navigation Toolbox
ver('signal')         % Signal Processing Toolbox
```

**Instalación de dependencias adicionales:**
```matlab
% Point Cloud Library para MATLAB
addpath('external/pcl_matlab');

% Velodyne driver (si se usa hardware real)
addpath('external/velodyne_driver');
```

### Configuración de Hardware

**Conexión Velodyne VLP-16:**
```matlab
% Configurar conexión Ethernet
vlp16 = velodynelidar('VLP16', '192.168.1.201');

% Parámetros de captura
vlp16.Duration = inf;           % Captura continua
vlp16.ReturnType = 'Strongest'; % Tipo de retorno
```

**Configuración RTK-GPS:**
```matlab
% Configurar puerto serie para receptor GPS
gps_port = serialport("COM3", 115200);
configureTerminator(gps_port, "LF");
```

## 🧪 Experimentos y Resultados

### Experimento 1: Evaluación de Odometría LiDAR

**Objetivo:** Comparar precisión de ICP vs NDT en diferentes entornos

```matlab
% Script principal: exp_01_icp_evaluation.m
function results = evaluate_lidar_odometry()
    datasets = {'urban', 'parking', 'highway'};
    algorithms = {'icp', 'ndt'};
    
    for i = 1:length(datasets)
        for j = 1:length(algorithms)
            [error_pos, error_rot, time] = run_odometry_test(...
                datasets{i}, algorithms{j});
            results(i,j) = struct('pos_rmse', error_pos, ...
                                 'rot_rmse', error_rot, ...
                                 'proc_time', time);
        end
    end
end
```

### Experimento 2: Análisis de Degradación GPS

**Objetivo:** Evaluar comportamiento del algoritmo bajo diferentes condiciones de señal GPS

```matlab
function analyze_gps_degradation()
    % Simular diferentes niveles de disponibilidad GPS
    gps_availability = [1.0, 0.8, 0.6, 0.4, 0.2]; % Porcentaje de disponibilidad
    
    for availability = gps_availability
        mask = generate_gps_dropout_mask(availability);
        trajectory_estimated = run_fusion_algorithm(data, mask);
        error_metrics = compute_trajectory_error(trajectory_estimated, ground_truth);
        
        plot_results(availability, error_metrics);
    end
end
```

### Experimento 3: Optimización de Parámetros

**Objetivo:** Optimizar parámetros del filtro de Kalman para mejor rendimiento

```matlab
% Optimización bayesiana de hiperparámetros
function optimal_params = optimize_fusion_parameters()
    % Definir espacio de búsqueda
    params_range = struct(...
        'process_noise_pos', [1e-4, 1e-1], ...
        'process_noise_rot', [1e-5, 1e-2], ...
        'gps_noise', [1e-3, 1e-1], ...
        'lidar_noise', [1e-3, 1e-1]);
    
    % Función objetivo: minimizar error RMS
    objective = @(params) evaluate_fusion_performance(params);
    
    % Ejecutar optimización
    optimal_params = bayesopt(objective, params_range, ...
        'MaxObjectiveEvaluations', 100);
end
```

## 📈 Objetivos de Investigación

### Métricas de Éxito Esperadas

**Precisión de localización:**
- Error de posición RMS < 10cm en condiciones normales de GPS
- Error de orientación < 1° en todos los ejes
- Mantenimiento de precisión <50cm durante pérdidas GPS de hasta 30 segundos

**Rendimiento computacional:**
- Procesamiento en tiempo real a frecuencia mínima de 5Hz
- Latencia total del pipeline < 200ms
- Escalabilidad para procesamiento en hardware embebido

**Robustez del sistema:**
- Operación estable en entornos urbanos complejos
- Recuperación automática tras pérdidas prolongadas de GPS
- Adaptación a diferentes condiciones meteorológicas

## 🔍 Algoritmo Principal de Fusión

### Implementación del EKF Pose Estimator

```matlab
classdef EKFPoseEstimator < handle
    properties
        state           % Estado del vehículo [9x1]
        covariance      % Matriz de covarianza [9x9]
        process_noise   % Matriz Q [9x9]
        dt              % Tiempo de muestreo
    end
    
    methods
        function obj = EKFPoseEstimator(initial_state, initial_cov)
            obj.state = initial_state;
            obj.covariance = initial_cov;
            obj.dt = 0.1; % 10 Hz por defecto
        end
        
        function predict(obj, control_input)
            % Predicción basada en modelo de movimiento
            [obj.state, obj.covariance] = obj.ekf_predict(...
                obj.state, obj.covariance, control_input);
        end
        
        function update_gps(obj, gps_measurement, gps_noise)
            % Actualización con medición GPS
            H = [eye(3), zeros(3,6)]; % Matriz de observación
            obj.ekf_update(gps_measurement, H, gps_noise);
        end
        
        function update_lidar(obj, lidar_transform, lidar_noise)
            % Actualización con odometría LiDAR
            predicted_transform = obj.compute_predicted_transform();
            innovation = obj.transform_to_vector(...
                lidar_transform \ predicted_transform);
            
            H = obj.compute_lidar_jacobian();
            obj.ekf_update(innovation, H, lidar_noise);
        end
    end
end
```

### Sincronización Temporal de Sensores

```matlab
function [synced_gps, synced_lidar] = synchronize_sensors(gps_data, lidar_data)
    % Interpolar datos GPS a timestamps de LiDAR
    lidar_timestamps = [lidar_data.timestamp];
    gps_timestamps = [gps_data.timestamp];
    
    % Interpolación de posiciones GPS
    synced_gps = struct();
    synced_gps.position = interp1(gps_timestamps, ...
        [gps_data.position], lidar_timestamps, 'linear');
    synced_gps.timestamp = lidar_timestamps;
    
    % LiDAR ya está en la frecuencia objetivo
    synced_lidar = lidar_data;
    
    % Remover datos fuera del rango temporal común
    valid_range = (lidar_timestamps >= min(gps_timestamps)) & ...
                  (lidar_timestamps <= max(gps_timestamps));
    
    synced_gps.position = synced_gps.position(valid_range, :);
    synced_gps.timestamp = synced_gps.timestamp(valid_range);
    synced_lidar = synced_lidar(valid_range);
end
```

## 💻 Implementación en MATLAB

### Arquitectura del Código

El sistema está diseñado con una arquitectura modular que separa claramente el procesamiento de cada sensor y la lógica de fusión:

```matlab
% Clase principal del sistema de fusión
classdef RTKLidarFusion < handle
    properties (Access = private)
        ekf_estimator           % Filtro de Kalman Extendido
        lidar_processor        % Procesador de datos Velodyne
        gps_processor          % Procesador RTK-GPS
        calibration_params     % Parámetros de calibración extrínseca
        sync_buffer           % Buffer para sincronización temporal
    end
    
    methods (Access = public)
        function obj = RTKLidarFusion(config)
            obj.initialize_components(config);
        end
        
        function pose = process_sensors(obj, lidar_scan, gps_data, timestamp)
            % Pipeline principal de procesamiento
            pose = obj.run_fusion_step(lidar_scan, gps_data, timestamp);
        end
    end
end
```

### Procesamiento de Datos Velodyne VLP-16

```matlab
function relative_transform = compute_lidar_odometry(scan_current, scan_previous)
    % Preprocesamiento: filtrado y voxelización
    scan_current = preprocess_pointcloud(scan_current);
    scan_previous = preprocess_pointcloud(scan_previous);
    
    % Registro ICP con múltiples resoluciones
    initial_guess = eye(4);
    [transform_coarse, ~] = pcregistericp(scan_current, scan_previous, ...
        'InitialTransform', initial_guess, 'MaxIterations', 50);
    
    % Refinamiento con NDT
    relative_transform = ndt_registration(scan_current, scan_previous, ...
        transform_coarse);
    
    % Validación de la transformación
    if ~validate_transform(relative_transform)
        relative_transform = eye(4); % Fallback a identidad
    end
end

function scan_filtered = preprocess_pointcloud(scan_raw)
    % Remover puntos fuera del rango útil
    distance_mask = (scan_raw.Location(:,1).^2 + scan_raw.Location(:,2).^2) > 1.0;
    scan_filtered = select(scan_raw, distance_mask);
    
    % Voxelización para reducir densidad
    scan_filtered = pcdownsample(scan_filtered, 'gridAverage', 0.2);
    
    % Filtro estadístico para outliers
    scan_filtered = pcdenoise(scan_filtered);
end
```

### Integración RTK-GPS

```matlab
function [position, quality] = process_rtk_gps(nmea_sentence)
    % Parser de mensajes NMEA GGA
    if contains(nmea_sentence, '$GPGGA') || contains(nmea_sentence, '$GNGGA')
        fields = split(nmea_sentence, ',');
        
        % Extraer coordenadas
        lat_deg = parse_coordinate(fields{3}, fields{4});
        lon_deg = parse_coordinate(fields{5}, fields{6});
        altitude = str2double(fields{10});
        
        % Convertir a coordenadas UTM locales
        [x_utm, y_utm, zone] = deg2utm(lat_deg, lon_deg);
        position = [x_utm; y_utm; altitude];
        
        % Evaluar calidad de la señal
        fix_quality = str2double(fields{7});
        hdop = str2double(fields{9});
        quality = assess_gps_quality(fix_quality, hdop);
    else
        position = [];
        quality = 0;
    end
end

function quality_score = assess_gps_quality(fix_type, hdop)
    % RTK Fixed: máxima calidad
    if fix_type == 4
        quality_score = 1.0;
    % RTK Float: alta calidad
    elseif fix_type == 5
        quality_score = 0.8;
    % DGPS: calidad media
    elseif fix_type == 2 && hdop < 2.0
        quality_score = 0.6;
    % GPS estándar: baja calidad
    elseif fix_type == 1 && hdop < 5.0
        quality_score = 0.3;
    else
        quality_score = 0.0; % Sin señal útil
    end
end
```

### Core del Algoritmo de Fusión

```matlab
function pose_estimate = run_fusion_step(obj, lidar_scan, gps_data, timestamp)
    % 1. Sincronización temporal
    [synced_lidar, synced_gps] = obj.synchronize_measurements(...
        lidar_scan, gps_data, timestamp);
    
    % 2. Predicción EKF basada en modelo cinemático
    obj.ekf_estimator.predict(timestamp);
    
    % 3. Actualización con odometría LiDAR (siempre disponible)
    if ~isempty(synced_lidar)
        relative_motion = obj.lidar_processor.compute_odometry(synced_lidar);
        obj.ekf_estimator.update_lidar(relative_motion);
    end
    
    % 4. Actualización con GPS (cuando calidad es suficiente)
    if ~isempty(synced_gps) && synced_gps.quality > 0.5
        obj.ekf_estimator.update_gps(synced_gps.position, synced_gps.quality);
    end
    
    % 5. Obtener estimación final
    pose_estimate = obj.ekf_estimator.get_current_pose();
    
    % 6. Logging y visualización
    obj.log_fusion_state(pose_estimate, timestamp);
end
```

### Calibración Extrínseca Automática

```matlab
function T_lidar_to_gps = calibrate_extrinsic_transform(lidar_trajectory, gps_trajectory)
    % Alineación temporal usando correlación cruzada
    [lag, correlation] = xcorr(lidar_trajectory.timestamps, gps_trajectory.timestamps);
    [~, max_idx] = max(correlation);
    time_offset = lag(max_idx);
    
    % Sincronizar trayectorias
    gps_synced = interpolate_trajectory(gps_trajectory, ...
        lidar_trajectory.timestamps + time_offset);
    
    % Estimación de transformación usando SVD
    lidar_points = lidar_trajectory.positions;
    gps_points = gps_synced.positions;
    
    % Centrar datos
    lidar_centroid = mean(lidar_points, 2);
    gps_centroid = mean(gps_points, 2);
    
    lidar_centered = lidar_points - lidar_centroid;
    gps_centered = gps_points - gps_centroid;
    
    % Calcular matriz de rotación y traslación
    H = lidar_centered * gps_centered';
    [U, ~, V] = svd(H);
    R = V * U';
    t = gps_centroid - R * lidar_centroid;
    
    % Construir matriz de transformación homogénea
    T_lidar_to_gps = [R, t; 0, 0, 0, 1];
end
```

## 🎯 Entregables Esperados

### Productos Técnicos

1. **Librería MATLAB** para fusión RTK-GPS/LiDAR
2. **Dataset anotado** con ground truth para validación
3. **Benchmark** comparativo de algoritmos de fusión
4. **Documentación técnica** completa del algoritmo

### Productos Académicos

1. **Reporte de estancia** (40-60 páginas)
2. **Artículo científico** para conferencia/revista
3. **Presentación técnica** para defensa de estancia
4. **Código documentado** en repositorio público

## 📊 Métricas de Éxito

### Objetivos Cuantitativos

- **Precisión de posición**: <10cm RMS en condiciones normales
- **Precisión de orientación**: <1° RMS en todos los ejes
- **Frecuencia de procesamiento**: >5Hz en tiempo real
- **Robustez**: <50cm error durante pérdidas GPS de 30s

### Objetivos Cualitativos

- Algoritmo robusto ante condiciones adversas
- Código modular y reutilizable
- Documentación clara para futura investigación
- Contribución al estado del arte en fusión sensorial

## 📚 Referencias Bibliográficas

### Papers Fundamentales

1. **Thrun, S.** (2002). "Robotic mapping: A survey." *Exploring artificial intelligence in the new millennium*, 1-35.

2. **Durrant-Whyte, H., & Bailey, T.** (2006). "Simultaneous localization and mapping: part I." *IEEE robotics & automation magazine*, 13(2), 99-110.

3. **Zhang, J., & Singh, S.** (2014). "LOAM: Lidar Odometry and Mapping in Real-time." *Robotics: Science and Systems*.

### Fusión Sensorial

4. **Gao, Y., et al.** (2018). "A robust INS/GPS/LiDAR-SLAM integrated navigation system for autonomous vehicles." *IEEE Transactions on Vehicular Technology*.

5. **Qin, T., et al.** (2018). "VINS-Mono: A robust and versatile monocular visual-inertial state estimator." *IEEE Transactions on Robotics*.

### RTK-GPS Technical

6. **Takasu, T., & Yasuda, A.** (2009). "Development of the low-cost RTK-GPS receiver with an open source program package RTKLIB." *International symposium on GPS/GNSS*.

## 📞 Contacto y Supervisión

**Investigador Principal:**
- **Nombre**: [Tu nombre]
- **Institución**: [Universidad/Centro de investigación]
- **Email**: [tu.email@institucion.edu]

**Supervisor de Estancia:**
- **Dr./Dra.**: [Nombre del supervisor]
- **Especialidad**: Robótica y Sistemas Autónomos
- **Email**: [supervisor@institucion.edu]

**Colaboradores:**
- **Laboratorio**: [Nombre del laboratorio]
- **Grupo de investigación**: [Nombre del grupo]

---

**📄 Licencia:** Este proyecto de investigación está bajo licencia académica. Los resultados y código pueden ser utilizados para fines educativos y de investigación con la debida atribución.

*Última actualización: Septiembre 2025*
