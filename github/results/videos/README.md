# 📹 Videos de Resultados

Este directorio contiene los videos demostrativos del sistema de fusión RTK-GPS + LiDAR.

## 📁 Estructura de Archivos

```
videos/
├── mapeo_v1.mp4              # Video del proceso de mapeo (Vuelta 1)
├── localizacion_v2.mp4       # Video de localización precisa (Vuelta 2)
└── demo_completo.mp4         # Demo completo del sistema
```

## 🎬 Videos Disponibles

### 1. **mapeo_v1.mp4** - Proceso de Mapeo 3D
- **Duración**: ~2 minutos
- **Contenido**: 
  - Construcción incremental del mapa 3D
  - Fusión 50% RTK + 50% LiDAR
  - Visualización de nubes de puntos en tiempo real
  - Trayectoria de mapeo
- **Vista**: Top-down + perspectiva 3D

### 2. **localizacion_v2.mp4** - Localización de Alta Precisión
- **Duración**: ~2 minutos
- **Contenido**:
  - Localización sobre mapa pre-construido
  - Fusión 85% RTK + 15% LiDAR
  - Corrección de deriva en tiempo real
  - Comparación con trayectoria de referencia
- **Vista**: Top-down + perspectiva 3D con overlay

### 3. **demo_completo.mp4** - Demostración Completa
- **Duración**: ~4 minutos
- **Contenido**:
  - Pipeline completo: captura → mapeo → localización
  - Comparación lado a lado Vuelta 1 vs Vuelta 2
  - Métricas de precisión en tiempo real
  - Manejo de pérdida de señal GPS
- **Vista**: Split screen con múltiples perspectivas

## 🎥 Cómo Generar los Videos

### Opción 1: Desde MATLAB (Recomendado)

```matlab
% Ejecutar el script principal con exportación de video
lidar_slam_3d_rtk_professional_v_clusters_mejorado();

% Los videos se generan automáticamente en:
% results/videos/
```

### Opción 2: Usando el Script de Exportación

```matlab
% Script dedicado para generar videos
export_v1_video();  % Ya existe en el repositorio

% O el script rápido:
generar_video_mapeo_v1_rapido();
```

### Opción 3: Generación Manual

```matlab
% 1. Ejecutar el procesamiento
run('lidar_slam_3d_rtk_professional_v_clusters_mejorado.m');

% 2. Crear el video writer
v = VideoWriter('results/videos/mapeo_v1.mp4', 'MPEG-4');
v.FrameRate = 10;
v.Quality = 95;
open(v);

% 3. Capturar frames de la visualización
for i = 1:num_frames
    frame = getframe(gcf);
    writeVideo(v, frame);
end

% 4. Cerrar el video
close(v);
```

## 📊 Configuración Recomendada

### Resolución y Calidad
```matlab
% Para videos de alta calidad:
v.FrameRate = 30;      % 30 fps (suave)
v.Quality = 95;        % Calidad 95/100
v.VideoFormat = 'H.264';

% Resolución de figura:
set(gcf, 'Position', [100, 100, 1920, 1080]);  % Full HD
```

### Tamaño de Archivo Esperado
```
mapeo_v1.mp4         (~50 MB)
localizacion_v2.mp4  (~50 MB)
demo_completo.mp4    (~100 MB)
```

## 🔗 Enlaces en el README

Los videos se enlazan en el README principal como:

```markdown
[![Ver Video](https://img.shields.io/badge/▶️_Ver_Video-Mapeo_3D-blue?style=for-the-badge)](./results/videos/mapeo_v1.mp4)
```

## 📝 Notas

- **Formato**: MP4 (H.264) para máxima compatibilidad
- **Compresión**: Balance entre calidad y tamaño
- **Hosting**: 
  - Archivos pequeños (<100MB): Directamente en el repositorio
  - Archivos grandes (>100MB): Considera usar GitHub LFS o YouTube/Vimeo

---

## 🚀 Próximos Pasos

Si aún no has generado los videos:

1. Ejecuta el procesamiento principal:
   ```matlab
   lidar_slam_3d_rtk_professional_v_clusters_mejorado();
   ```

2. Los videos se generarán automáticamente en este directorio

3. Si no se generan automáticamente, usa:
   ```matlab
   export_v1_video();
   generar_video_mapeo_v1_rapido();
   ```

---

<div align="center">

**Nota**: Los videos pueden tardar varios minutos en generarse dependiendo de la duración del dataset.

</div>
