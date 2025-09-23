
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

### Pipeline de Captura de Datos RTK-LiDAR

El sistema desarrollado para la captura sincronizada de datos RTK-GPS y Velodyne VLP-16 utiliza un enfoque de streaming en tiempo real:

```matlab
% Configuración del sistema de captura
rtkPort = 'COM5';           % Puerto del receptor RTK
rtkBaud = 115200;           % Velocidad de comunicación
lidar = velodynelidar('VLP16');  % Objeto LiDAR Velodyne

% Bucle principal de captura sincronizada
for k = 1:1e6
    % Lectura frame LiDAR con timestamp
    [pc, t] = read(lidar,1);
    frames{k} = pc;
    timestamps(k,1) = t;
    
    % Lectura RTK simultánea
    rtk = struct('lat',nan,'lon',nan,'alt',nan);
    while s.NumBytesAvailable > 0
        line = readline(s);
        if startsWith(line,"$GPGGA") || startsWith(line,"$GNGGA")
            rtk = parseNMEA_GGA(line, rtk);
        end
    end
    
    % Almacenamiento sincronizado
    K(k) = struct('t',t, 'frame',pc, 'lat',rtk.lat, 'lon',rtk.lon, 'alt',rtk.alt);
end
```

### Estructura de Datos Capturados (.mat)

Los datos se almacenan en archivos .mat con la siguiente estructura:

```matlab
% Variables principales en el archivo .mat
frames      % Cell array: {pointCloud_1, pointCloud_2, ..., pointCloud_n}
timestamps  % Array datetime: [t1; t2; ...; tn] - timestamps LiDAR
lat         % Array double: [lat1; lat2; ...; latn] - latitudes RTK  
lon         % Array double: [lon1; lon2; ...; lonn] - longitudes RTK
alt         % Array double: [alt1; alt2; ...; altn] - altitudes RTK
rtkTime     % Array datetime: [rt1; rt2; ...; rtn] - timestamps RTK
K           % Struct array: K(i) = {t, frame, lat, lon, alt} - datos consolidados
```

### Procesamiento de Datos RTK-GPS

```matlab
function rtk = parseNMEA_GGA(line, rtk)
    % Parser de mensajes NMEA GGA para extraer coordenadas RTK
    p = split(string(line), ",");
    if numel(p) < 10, return; end
    
    latStr = p{3}; latHem = p{4};   % ddmm.mmmm y hemisferio N/S
    lonStr = p{5}; lonHem = p{6};   % dddmm.mmmm y hemisferio E/W  
    altStr = p{10};                 % altitud en metros
    
    if strlength(latStr) >= 4 && strlength(lonStr) >= 5
        latVal = nmeaToDeg(latStr,true);    % conversión a grados decimales
        lonVal = nmeaToDeg(lonStr,false);
        
        % Aplicar signos según hemisferio
        if strcmpi(latHem,'S'), latVal = -latVal; end
        if strcmpi(lonHem,'W'), lonVal = -lonVal; end
        
        rtk.lat = latVal;
        rtk.lon = lonVal;
    end
    
    % Procesamiento de altitud
    a = str2double(altStr);
    if ~isnan(a), rtk.alt = a; end
end
```

### Algoritmo de Fusión Post-Procesamiento

Una vez capturados los datos sincronizados, el algoritmo de fusión procesa el archivo .mat:

```matlab
function pose_trajectory = process_captured_data(mat_filename)
    % Cargar datos capturados
    load(mat_filename, 'frames', 'timestamps', 'lat', 'lon', 'alt', 'K');
    
    % Inicializar estimador de pose
    pose_estimator = EKFPoseEstimator();
    pose_trajectory = [];
    
    for i = 2:length(frames)
        % Odometría LiDAR entre frames consecutivos
        relative_transform = compute_lidar_odometry(frames{i}, frames{i-1});
        
        % Coordenadas RTK válidas para este frame
        if ~isnan(lat(i)) && ~isnan(lon(i)) && ~isnan(alt(i))
            % Convertir coordenadas geodésicas a UTM local
            [x_utm, y_utm] = deg2utm(lat(i), lon(i));
            gps_position = [x_utm; y_utm; alt(i)];
            gps_available = true;
        else
            gps_position = [];
            gps_available = false;
        end
        
        % Aplicar fusión EKF
        pose_estimator.predict(timestamps(i));
        pose_estimator.update_lidar(relative_transform);
        
        if gps_available
            pose_estimator.update_gps(gps_position);
        end
        
        % Guardar pose estimada
        current_pose = pose_estimator.get_current_pose();
        pose_trajectory = [pose_trajectory; current_pose'];
    end
end
```

### Sincronización Temporal y Validación

```matlab
function [valid_indices, sync_quality] = validate_synchronization(timestamps, rtkTime, K)
    % Análisis de calidad de sincronización temporal
    valid_indices = [];
    sync_quality = [];
    
    for i = 1:length(K)
        % Verificar disponibilidad de datos RTK
        has_rtk = ~isnan(K(i).lat) && ~isnan(K(i).lon) && ~isnan(K(i).alt);
        
        % Verificar calidad del frame LiDAR
        num_points = size(K(i).frame.Location, 1);
        has_sufficient_points = num_points > 1000;  % mínimo 1000 puntos
        
        % Calcular diferencia temporal entre LiDAR y RTK
        if has_rtk && i <= length(rtkTime)
            time_diff = abs(seconds(timestamps(i) - rtkTime(i)));
            temporal_sync = time_diff < 0.1;  % sincronización < 100ms
        else
            temporal_sync = false;
        end
        
        % Frame válido si cumple criterios mínimos
        if has_sufficient_points
            valid_indices(end+1) = i;
            
            % Calidad basada en disponibilidad RTK y sincronización
            if has_rtk && temporal_sync
                sync_quality(end+1) = 1.0;  % calidad máxima
            elseif has_rtk
                sync_quality(end+1) = 0.7;  % RTK disponible pero desincronizado
            else
                sync_quality(end+1) = 0.3;  % solo LiDAR disponible
            end
        end
    end
end
```

### Herramientas de Análisis y Visualización

```matlab
function analyze_dataset(mat_filename)
    load(mat_filename, 'K', 'timestamps');
    
    % Estadísticas del dataset
    total_frames = length(K);
    frames_with_rtk = sum(~isnan([K.lat]));
    rtk_coverage = frames_with_rtk / total_frames * 100;
    
    fprintf('📊 Análisis del Dataset:\n');
    fprintf('   Total de frames: %d\n', total_frames);
    fprintf('   Frames con RTK: %d (%.1f%%)\n', frames_with_rtk, rtk_coverage);
    
    % Análisis de calidad de puntos LiDAR
    point_counts = arrayfun(@(x) size(x.frame.Location,1), K);
    fprintf('   Puntos LiDAR promedio: %.0f ± %.0f\n', mean(point_counts), std(point_counts));
    
    % Análisis temporal
    duration = timestamps(end) - timestamps(1);
    avg_frequency = total_frames / seconds(duration);
    fprintf('   Duración: %.1f segundos\n', seconds(duration));
    fprintf('   Frecuencia promedio: %.1f Hz\n', avg_frequency);
    
    % Visualización de trayectoria RTK (cuando disponible)
    valid_rtk = ~isnan([K.lat]) & ~isnan([K.lon]);
    if sum(valid_rtk) > 10
        figure;
        plot([K(valid_rtk).lon], [K(valid_rtk).lat], 'b.-', 'LineWidth', 2);
        xlabel('Longitud [°]'); ylabel('Latitud [°]');
        title('Trayectoria RTK-GPS');
        grid on; axis equal;
    end
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



**📄 Licencia:** Este proyecto de investigación está bajo licencia académica. Los resultados y código pueden ser utilizados para fines educativos y de investigación con la debida atribución.

*Última actualización: Septiembre 2025*
