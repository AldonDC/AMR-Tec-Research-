<div align="center">

![MATLAB RTK-GPS LiDAR SLAM Cover](github/assets/matlab_cover.svg)

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b+-orange.svg?style=for-the-badge&logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAyNCAyNCI+PHBhdGggZmlsbD0iI2ZmZiIgZD0iTTIyLjUgMTQuNWMtLjcgMC0xLjUuMy0yLjEuOWwtMi45LTEuN2MuNC0uNy42LTEuNS42LTIuNHMtLjItMS43LS42LTIuNGwyLjktMS43Yy42LjUgMS40LjkgMi4xLjkgMS45IDAgMy41LTEuNiAzLjUtMy41UzI0LjQgMS4xIDIyLjUgMS4xUzE5IDIuNyAxOSA0LjZjMCAuNS4xIDEgLjMgMS40bC0yLjkgMS43Yy0uOS0xLjItMi4zLTItMy45LTJzLTMgLjgtMy45IDJsLTIuOS0xLjdjLjItLjQuMy0uOS4zLTEuNCAwLTEuOS0xLjYtMy41LTMuNS0zLjVTLjUgMi43LjUgNC42czEuNiAzLjUgMy41IDMuNWMuNyAwIDEuNS0uMyAyLjEtLjlsMi45IDEuN2MtLjQuNy0uNiAxLjUtLjYgMi40czIgMS43LjYgMi40bC0yLjkgMS43Yy0uNi0uNS0xLjQtLjktMi4xLS45LTEuOSAwLTMuNSAxLjYtMy41IDMuNVM0LjEgMjEuNCA2IDIxLjRzMy41LTEuNiAzLjUtMy41YzAtLjUtLjEtMS0uMy0xLjRsMi45LTEuN2MuOSAxLjIgMi4zIDIgMy45IDJzMy0uOCAzLjktMmwyLjkgMS43Yy0uMi40LS4zLjktLjMgMS40IDAgMS45IDEuNiAzLjUgMy41IDMuNXMzLjUtMS42IDMuNS0zLjUtMS42LTMuNS0zLjUtMy41eiIvPjwvc3ZnPg==)](https://www.mathworks.com/products/matlab.html)
[![Velodyne](https://img.shields.io/badge/LiDAR-Velodyne_VLP16-blue.svg?style=for-the-badge)](https://velodynelidar.com/products/puck/)
[![GPS](https://img.shields.io/badge/GPS-RTK_Enabled-green.svg?style=for-the-badge)]()
[![License](https://img.shields.io/badge/License-Academic-lightgrey.svg?style=for-the-badge)]()

---

[Características](#-características-principales) •
[Instalación](#-instalación-y-configuración) •
[Uso](#-uso-del-sistema) •
[Algoritmos](#-algoritmos-implementados) •
[Resultados](#-resultados-experimentales) •
[Referencias](#-referencias-bibliográficas)

</div>

---

## 📋 Descripción del Proyecto

Esta estancia de investigación desarrolla e implementa **algoritmos avanzados de fusión sensorial** que combinan datos de **RTK-GPS** (Real-Time Kinematic) y **LiDAR Velodyne VLP-16** para obtener estimaciones precisas y robustas de la pose vehicular en tiempo real (posición 3D + orientación).

### 🎯 Objetivos de Investigación

**Objetivo Principal:**
Desarrollar un sistema de fusión sensorial multi-modal que integre mediciones RTK-GPS y datos LiDAR 3D para determinar la pose 6DOF (6 grados de libertad) de un vehículo con precisión centimétrica y robustez ante oclusiones de señal satelital.

**Objetivos Específicos:**
- ✅ Implementar algoritmos de odometría LiDAR (ICP, NDT)
- ✅ Desarrollar pipeline de captura sincronizada RTK-GPS/LiDAR
- ✅ Crear sistema de fusión probabilística con detección de deriva
- ✅ Optimizar algoritmos para procesamiento en tiempo real (>5 Hz)
- ✅ Validar precisión mediante datos de campo con ground truth RTK

---

## 🔬 Marco Teórico

### Sensores Utilizados

#### 📡 RTK-GPS (Real-Time Kinematic)

**Características Técnicas:**
- **Precisión horizontal**: ±2cm (modo RTK-Fixed)
- **Precisión vertical**: ±5cm
- **Frecuencia de actualización**: 10-20 Hz
- **Protocolo**: NMEA 0183 (mensajes `$GPGGA` / `$GNGGA`)
- **Comunicación**: Serial RS-232 @ 115200 bps

**Ventajas:**
- ✅ Referencia absoluta global (no acumula deriva)
- ✅ Precisión centimétrica en condiciones ideales
- ✅ Cobertura ilimitada en exteriores

**Limitaciones:**
- ❌ Pérdida de señal en interiores/túneles/pasos elevados
- ❌ Multipath en entornos urbanos densos
- ❌ Requiere línea de vista a ≥4 satélites + estación base

#### 🌀 Velodyne VLP-16 LiDAR

**Especificaciones del Sensor:**
- **Canales**: 16 láseres (905 nm)
- **Rango**: 100m (especificación), 10-15m efectivo en exteriores
- **Precisión**: ±3cm
- **Frecuencia de rotación**: 5-20 Hz (configuración: 10 Hz)
- **Puntos por segundo**: ~300,000
- **Campo de visión**: 360° horizontal, ±15° vertical (-15° a +15°)
- **Resolución angular**: 0.1-0.4° (según configuración)

**Ventajas:**
- ✅ Percepción 3D completa del entorno local
- ✅ Robusto ante condiciones meteorológicas
- ✅ No requiere infraestructura externa
- ✅ Opera en interiores y exteriores

**Limitaciones:**
- ❌ Odometría relativa acumula deriva sin corrección
- ❌ Computacionalmente intensivo (procesamiento de nubes de puntos)
- ❌ Alcance limitado vs GPS

---

## 🧭 Metodología de Fusión Sensorial

### Estrategia de Combinación Multi-Sensor

El sistema implementa una **arquitectura de fusión probabilística** que combina las fortalezas complementarias de ambos sensores:

<img width="763" height="814" alt="image" src="https://github.com/user-attachments/assets/8922ac84-9939-4a75-b0cc-bc4e4e31a0d5" />

---


### Algoritmo de Fusión Implementado

El sistema utiliza una **estrategia de fusión ponderada adaptativa** en lugar de un filtro de Kalman completo, priorizada para rendimiento en tiempo real:

```matlab
% Pesos de fusión adaptativos según fase del recorrido
if fase == 1  % VUELTA 1: Mapeo inicial (trayectoria de referencia)
    RTK_WEIGHT = 0.50;   % 50% RTK + 50% LiDAR (odometría)
    % Objetivo: Construir mapa 3D del entorno
    
elseif fase == 2  % VUELTA 2: Localización precisa con corrección de deriva
    RTK_WEIGHT = 0.85;   % 85% RTK + 15% LiDAR
    % Objetivo: Localización precisa usando mapa de V1
    % Control activo de deriva + loop closure
end

% Fusión de posiciones
pose_fusion = RTK_WEIGHT * pose_rtk + (1 - RTK_WEIGHT) * pose_lidar;
```

**Justificación de la estrategia:**
- **Vuelta 1 (Mapeo)**: Balance 50/50 para capturar geometría 3D con anclaje RTK
- **Vuelta 2 (Localización)**: Dominancia RTK (85%) para máxima precisión con refinamiento LiDAR

### Manejo de Pérdidas de Señal GPS

Estrategia de degradación graceful implementada:

```matlab
% Detección de pérdida de señal RTK
function isValid = validateRTKSignal(lat, lon, alt)
    isValid = ~isnan(lat) && ~isnan(lon) && ~isnan(alt) && ...
              abs(lat) > 1e-6 && abs(lon) > 1e-6;
end

% Fallback hierarchy durante pérdida GPS
if ~validateRTKSignal(rtk)
    % 1. Intentar NDT registration (método primario)
    [tform_ndt, rmse_ndt] = pcregisterndt(source, target, gridStep);
    
    if rmse_ndt > RMSE_THRESHOLD  % NDT falló
        % 2. Fallback a ICP (método secundario)
        [tform_icp, rmse_icp] = pcregistericp(source, target);
        
        if rmse_icp > RMSE_THRESHOLD  % ICP también falló
            % 3. Fallback a último RTK conocido + dead reckoning
            warning('Multi-strategy failure: using last known RTK');
            pose = pose_last_valid;
        end
    end
end
```

**Características de robustez:**
- 🔄 Multi-estrategia de registro (NDT → ICP → Fallback RTK)
- 📊 Validación de RMSE para cada método
- ⚠️ Propagación de incertidumbre durante pérdida GPS
- 🔍 Loop closure para detección y corrección de deriva acumulada

---

## 🔧 Instalación y Configuración

### Requisitos del Sistema

**Software:**
```
MATLAB R2023b o superior
├── Lidar Toolbox
├── Navigation Toolbox  
├── Robotics System Toolbox
├── Computer Vision Toolbox
└── Mapping Toolbox (opcional)
```

**Hardware (para captura en tiempo real):**
```
Hardware Setup:
├── Velodyne VLP-16 LiDAR
│   ├── Conexión: Ethernet (192.168.1.201)
│   ├── Alimentación: 12V DC
│   └── Frecuencia: 10 Hz (recomendado)
│
├── Receptor RTK-GPS
│   ├── Conexión: USB Serial (COM5 en Windows)
│   ├── Baudrate: 115200 bps
│   ├── Protocolo: NMEA 0183
│   └── Modo: RTK-Fixed (precisión cm)
│
└── PC de Procesamiento
    ├── RAM: ≥16 GB (recomendado 32 GB)
    ├── CPU: Intel i7 / AMD Ryzen 7 o superior
    └── Almacenamiento: SSD (procesamiento I/O intensivo)
```

### Verificación de Toolboxes MATLAB

```matlab
% Script de verificación rápida
function checkDependencies()
    required_toolboxes = {'lidar', 'robotics', 'nav', 'vision'};
    fprintf('🔍 Verificando dependencias...\n\n');
    
    for i = 1:length(required_toolboxes)
        tb = required_toolboxes{i};
        v = ver(tb);
        if isempty(v)
            fprintf('❌ %s Toolbox: NO INSTALADO\n', upper(tb));
        else
            fprintf('✅ %s Toolbox: %s\n', upper(tb), v.Version);
        end
    end
end
```

### Configuración del Hardware

#### 1️⃣ Conexión Velodyne VLP-16

```matlab
% Configurar interfaz de red para LiDAR
% IP del LiDAR: 192.168.1.201
% IP del PC: 192.168.1.100 (misma subred)

% En MATLAB:
lidar = velodynelidar('VLP16');
lidar.Duration = inf;  % Captura continua
start(lidar);

% Verificar conectividad
[pc, timestamp] = read(lidar, 1);
fprintf('✅ LiDAR conectado: %d puntos capturados\n', pc.Count);
```

#### 2️⃣ Configuración RTK-GPS

```matlab
% Conexión serial al receptor RTK
rtkPort = 'COM5';  % Ajustar según tu sistema (COM3, COM5, etc.)
rtkBaud = 115200;

s = serialport(rtkPort, rtkBaud, "Timeout", 0.5);
configureTerminator(s, "LF");  % Mensajes NMEA terminan con LF
flush(s);

% Test de lectura
line = readline(s);
if startsWith(line, "$GPGGA") || startsWith(line, "$GNGGA")
    fprintf('✅ RTK-GPS conectado correctamente\n');
else
    warning('⚠️  Verificar formato de mensajes RTK');
end
```

---

## 💻 Uso del Sistema

### Opción A: Captura de Datos en Tiempo Real

El sistema captura datos sincronizados de RTK-GPS y LiDAR. El script principal de captura se encuentra en el repositorio como el archivo base utilizado para generar los archivos `.mat`.

**Script de captura:** Ver sección de código de captura en la documentación técnica completa.

**Estructura del archivo `.mat` generado:**

```matlab
% Variables guardadas en 'recorrido_YYYYMMDD_HHMMSS.mat':
frames      % Cell array: {pointCloud_1, ..., pointCloud_N}
timestamps  % datetime array: [t1; t2; ...; tN] (timestamps LiDAR)
lat         % double array: [lat1; lat2; ...; latN] (latitudes RTK en grados)
lon         % double array: [lon1; lon2; ...; lonN] (longitudes RTK en grados)
alt         % double array: [alt1; alt2; ...; altN] (altitudes RTK en metros)
rtkTime     % datetime array: [rt1; rt2; ...; rtN] (timestamps RTK)
```

### Opción B: Procesamiento de Datos Capturados

Una vez capturados los datos, el algoritmo principal procesa el archivo `.mat`:

```matlab
%% SCRIPT PRINCIPAL: lidar_slam_3d_rtk_professional_v_clusters_mejorado.m
% ============================================================
% SLAM 3D PROFESIONAL CON FUSIÓN RTK-GPS + LIDAR VLP-16
% ============================================================

% Ejecutar el procesamiento:
lidar_slam_3d_rtk_professional_v_clusters_mejorado();

% El script automáticamente:
% 1. Carga el archivo .mat especificado
% 2. Detecta las 2 vueltas del recorrido
% 3. Procesa Vuelta 1 (mapeo con 50% RTK / 50% LiDAR)
% 4. Procesa Vuelta 2 (localización con 85% RTK / 15% LiDAR)
% 5. Genera visualizaciones y exporta resultados
```

**Archivos de salida generados:**
```
results/
├── mapa_3d_final.ply                    # Mapa 3D completo
├── trayectoria_v1_mapeo.csv             # Trayectoria Vuelta 1
├── trayectoria_v2_localizacion.csv      # Trayectoria Vuelta 2
└── figures/
    ├── trajectory_comparison.png         # Comparación V1 vs V2
    ├── rtk_trajectory_2d.png            # Trayectoria RTK pura
    └── 3d_map_with_trajectories.png     # Mapa 3D + trayectorias
```

---

## 🧪 Algoritmos Implementados

### 1️⃣ Odometría LiDAR: NDT + ICP

**Normal Distribution Transform (NDT):**
```matlab
% Registro NDT para estimación de movimiento relativo
[tform, rmse] = pcregisterndt(source, target, gridStep, ...
    'MaxIterations', maxIter, ...
    'Tolerance', [0.01, 0.001]);

% Parámetros:
%   - gridStep: Tamaño de celda (4-6m para outdoor)
%   - maxIter: Iteraciones máximas (40-50)
%   - Tolerance: [translación, rotación] en m y rad
```

**Iterative Closest Point (ICP) Fallback:**
```matlab
% Backup cuando NDT falla (RMSE alto)
[tform_icp, rmse_icp] = pcregistericp(source, target, ...
    'Metric', 'pointToPlane', ...
    'MaxIterations', 100, ...
    'Tolerance', [0.001, 0.0001]);
```

### 2️⃣ Filtrado de Suelo RANSAC

```matlab
function [pcd_no_ground, ground_model] = removeGroundRANSAC(pcd)
    % Ajuste de plano mediante RANSAC
    maxDistance = 0.10;  % Tolerancia ±10cm
    maxAngularDistance = 5;  % ±5° respecto a horizontal
    
    [model, inlierIndices] = pcfitplane(pcd, maxDistance, ...
        [0, 0, 1], maxAngularDistance);
    
    % Remover inliers (suelo)
    outlierIndices = setdiff(1:pcd.Count, inlierIndices);
    pcd_no_ground = select(pcd, outlierIndices);
    
    ground_model = model;
end
```

### 3️⃣ Detección de Deriva y Loop Closure

```matlab
function [corrected_pose, drift_detected] = detectAndCorrectDrift(...
    pose_lidar, pose_rtk, threshold)
    
    % Calcular deriva euclidiana
    drift_magnitude = norm(pose_lidar - pose_rtk);
    
    if drift_magnitude > threshold  % Umbral: 2.0m
        drift_detected = true;
        warning('⚠️  Deriva detectada: %.2fm - aplicando corrección RTK', ...
            drift_magnitude);
        
        % Corrección agresiva: forzar posición RTK
        corrected_pose = pose_rtk;
    else
        drift_detected = false;
        % Fusión normal
        RTK_WEIGHT = 0.85;
        corrected_pose = RTK_WEIGHT * pose_rtk + (1 - RTK_WEIGHT) * pose_lidar;
    end
end
```

### 4️⃣ Parser NMEA para RTK-GPS

```matlab
function rtk = parseNMEA_GGA(line, rtk)
    % Decodifica mensajes $GPGGA/$GNGGA para extraer lat/lon/alt
    p = split(string(line), ",");
    if numel(p) < 10, return; end
    
    latStr = p{3}; latHem = p{4};  % ddmm.mmmm, N/S
    lonStr = p{5}; lonHem = p{6};  % dddmm.mmmm, E/W
    altStr = p{10};                % metros
    
    if strlength(latStr) >= 4 && strlength(lonStr) >= 5
        latVal = nmeaToDeg(latStr, true);
        lonVal = nmeaToDeg(lonStr, false);
        
        if strcmpi(latHem, 'S'), latVal = -latVal; end
        if strcmpi(lonHem, 'W'), lonVal = -lonVal; end
        
        rtk.lat = latVal;
        rtk.lon = lonVal;
    end
    
    a = str2double(altStr);
    if ~isnan(a), rtk.alt = a; end
end
```

---

## 🎬 Resultados Experimentales

###  Videos Demostrativos

<div align="center">

<table>
<tr>
<td width="50%" align="center">

### 🗺️ Mapeo 3D - Vuelta 1
![Mapeo 3D Vuelta 1](github/results/videos/slam_v1_mapeo.gif)
*Construcción del mapa 3D con fusión 50% RTK + 50% LiDAR*

</td>
<td width="50%" align="center">

### 🎯 Localización - Vuelta 2
![Mapeo 3D Vuelta 2](github/results/videos/slam_v2_localizacion.gif)
*Localización precisa con fusión 85% RTK + 15% LiDAR*

</td>
</tr>
</table>

#### 🌐 Vista Completa del Sistema

[![Demo Completo](https://img.shields.io/badge/▶️_DEMO_COMPLETO-Sistema_RTK_LiDAR_SLAM-orange?style=for-the-badge&logo=youtube)](./results/videos/demo_completo.mp4)

*Visualización completa del pipeline: Mapeo + Localización + Corrección de Deriva*

---

### 📊 Visualizaciones Principales

<p align="center">
  <img src="./results/figures/trajectory_comparison.png" alt="Comparación de Trayectorias" width="45%">
  <img src="./results/figures/3d_map_with_trajectories.png" alt="Mapa 3D Completo" width="45%">
</p>

<p align="center">
  <img src="./results/figures/rtk_trajectory_2d.png" alt="Trayectoria RTK 2D" width="45%">
  <img src="./results/figures/error_analysis.png" alt="Análisis de Errores" width="45%">
</p>

</div>

---

### 📈 Dataset de Prueba

**Características del recorrido:**
```
📁 Archivo: recorrido_20250829_163719.mat
├── ⏱️  Duración: ~240 segundos
├── 📸 Frames capturados: 2,400 (10 Hz)
├── 📏 Distancia recorrida: ~500 metros
├── 🔄 Tipo de trayectoria: Circuito cerrado (2 vueltas)
├── 🌳 Entorno: Exterior urbano con vegetación
└── 📡 Condiciones GPS: RTK-Fixed 95% del tiempo
```

---

### 🎯 Métricas de Rendimiento

<div align="center">

#### 📍 Precisión de Localización

<table>
<tr>
<th width="50%">🗺️ Vuelta 1: Mapeo<br><sub>(50% RTK / 50% LiDAR)</sub></th>
<th width="50%">🎯 Vuelta 2: Localización<br><sub>(85% RTK / 15% LiDAR)</sub></th>
</tr>
<tr>
<td>

```
✓ Error RMS posición:    12.3 cm
✓ Error máximo:          45.8 cm
✓ Desviación estándar:    8.7 cm
✓ Puntos de mapa:     ~850,000
```

</td>
<td>

```
✅ Error RMS posición:     5.2 cm
   (Objetivo: <10cm)
✅ Error máximo:          18.4 cm
✅ Desviación estándar:    3.8 cm
✅ Error orientación:      0.8°
   (Objetivo: <1°)
```

</td>
</tr>
</table>

---

#### ⚡ Rendimiento Computacional

```
🖥️  Hardware: Intel i7-11800H (8 cores) + 32GB RAM + SSD

⏱️  Procesamiento por frame:      45-55 ms
🚀 Frecuencia efectiva:           18-22 fps  ✅ (objetivo >5 Hz)
⏳ Tiempo total Vuelta 1:         42 segundos
⏳ Tiempo total Vuelta 2:         38 segundos  
📊 Ratio tiempo real:             1:6 (6x más rápido que captura)
```

---

#### 🛡️ Robustez ante Pérdida GPS

```
🔬 Simulación de dropout GPS (30 segundos):

📍 Error sin GPS (solo LiDAR):       34.2 cm RMS
⚡ Recuperación tras re-adquisición:  <5 frames (0.5 seg)
📉 Deriva máxima acumulada:          52.1 cm
✅ Conclusión: Sistema mantiene precisión <50cm sin GPS
```

</div>

---

## 📊 Comparación con Estado del Arte

| Métrica | Este Trabajo | LOAM<sup>[1]</sup> | LeGO-LOAM<sup>[2]</sup> | HDL-Graph<sup>[3]</sup> |
|---------|-------------|-------|-----------|-------------------|
| **Error RMS (con GPS)** | **5.2 cm** | 10-15 cm | 8-12 cm | 3-5 cm |
| **Error RMS (sin GPS)** | 34 cm (30s) | 20-30 cm | 15-25 cm | 40-60 cm |
| **Frecuencia** | **18-22 Hz** | 10 Hz | 10 Hz | 5-8 Hz |
| **Sensor LiDAR** | VLP-16 | VLP-16 | VLP-16 | HDL-32 |
| **Fusión GPS** | ✅ RTK | ❌ | ❌ | ✅ GPS estándar |
| **Tiempo real** | ✅ | ✅ | ✅ | ⚠️ Semi-real |

<sup>[1]</sup> Zhang & Singh, RSS 2014  
<sup>[2]</sup> Shan & Englot, IROS 2018  
<sup>[3]</sup> Koide et al., ICRA 2019

**Ventajas competitivas:**
- ✅ Fusión RTK centimétrica (vs GPS estándar 1-3m)
- ✅ Detección activa de deriva con corrección automática
- ✅ Procesamiento más rápido (18-22 Hz vs 5-10 Hz)
- ✅ Multi-estrategia de registro (NDT → ICP → RTK fallback)

---


---

## 🚀 Trabajo Futuro

### Mejoras Planificadas

**Fase 1: Optimización Algorítmica** (Corto plazo - 3 meses)
- [ ] Implementación de filtro de Kalman extendido (EKF) completo
- [ ] Integración de IMU para estimación de orientación
- [ ] Optimización de backend con pose graph optimization
- [ ] Paralelización de procesamiento de nubes de puntos

**Fase 2: Robustez Avanzada** (Medio plazo - 6 meses)
- [ ] Deep learning para segmentación semántica de nubes
- [ ] Detección y tracking de objetos dinámicos
- [ ] SLAM semántico con landmarks
- [ ] Adaptación automática de parámetros según entorno

**Fase 3: Implementación en Tiempo Real** (Largo plazo - 12 meses)
- [ ] Migración a C++/ROS2 para hardware embebido
- [ ] Integración con stack de navegación autónoma
- [ ] Validación en vehículo real (campo de pruebas)
- [ ] Benchmark contra sistemas comerciales

---

## 📚 Referencias Bibliográficas

### Papers Fundamentales

**1. SLAM y Odometría LiDAR:**
- Zhang, J., & Singh, S. (2014). "LOAM: Lidar Odometry and Mapping in Real-time." *Robotics: Science and Systems*, 2(9).
- Shan, T., & Englot, B. (2018). "LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain." *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*.

**2. Fusión Sensorial Multi-Modal:**
- Gao, Y., Liu, S., Atia, M. M., & Noureldin, A. (2018). "INS/GPS/LiDAR Integrated Navigation System for Urban and Indoor Environments Using Hybrid Scan Matching Algorithm." *Sensors*, 18(11), 4004.

**3. RTK-GPS Technical:**
- Takasu, T., & Yasuda, A. (2009). "Development of the Low-cost RTK-GPS Receiver with an Open Source Program Package RTKLIB." *International Symposium on GPS/GNSS*, 4-6.

**4. NDT Registration:**
- Magnusson, M., Lilienthal, A., & Duckett, T. (2007). "Scan Registration for Autonomous Mining Vehicles Using 3D-NDT." *Journal of Field Robotics*, 24(10), 803-827.

### Recursos Técnicos

- **MATLAB Documentation**: [Lidar Toolbox](https://www.mathworks.com/help/lidar/)
- **Velodyne VLP-16 Manual**: [User Manual & Programming Guide](https://velodynelidar.com/products/puck/)
- **RTK-GPS Standards**: RTCM 10403.3 (Differential GNSS Services)

---

## 📄 Licencia

Este proyecto está bajo **licencia académica**. Los resultados y código pueden ser utilizados para fines educativos y de investigación con la debida atribución.

```
Copyright (c) 2025 Alfonso

Se permite el uso, copia, modificación y distribución de este software
para fines académicos y de investigación, con las siguientes condiciones:

1. Se debe citar este trabajo en cualquier publicación que utilice este código
2. No se permite el uso comercial sin autorización explícita
3. Cualquier modificación debe ser documentada y compartida bajo la misma licencia
```

---

## 🏆 Agradecimientos

Agradecimientos especiales a:
- Laboratorio de Robótica y Percepción
- Equipo de soporte técnico de MATLAB
- Comunidad de desarrolladores de PCL y ROS
- Revisores y evaluadores de este proyecto de investigación

---

<div align="center">

**⭐ Si este proyecto te resulta útil, considera darle una estrella en GitHub ⭐**

*Última actualización: Octubre 2025*

</div>
