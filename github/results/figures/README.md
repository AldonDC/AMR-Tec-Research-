# 📊 Figuras y Gráficas

Este directorio contiene las visualizaciones y gráficas generadas por el sistema de fusión RTK-GPS + LiDAR.

## 📁 Archivos Esperados

```
figures/
├── trajectory_comparison.png       # Comparación Vuelta 1 vs Vuelta 2
├── 3d_map_with_trajectories.png   # Mapa 3D completo con trayectorias
├── rtk_trajectory_2d.png          # Vista 2D de la trayectoria RTK
├── error_analysis.png             # Análisis de errores y precisión
├── drift_correction.png           # Visualización de corrección de deriva
└── performance_metrics.png        # Métricas de rendimiento
```

## 🎨 Descripción de las Figuras

### 1. **trajectory_comparison.png**
- **Contenido**: Comparación lado a lado de las trayectorias de Vuelta 1 (mapeo) y Vuelta 2 (localización)
- **Ejes**: X-Y en coordenadas UTM
- **Colores**: 
  - 🔴 Vuelta 1 (rojo)
  - 🔵 Vuelta 2 (azul)
  - 🟢 RTK Ground Truth (verde)

### 2. **3d_map_with_trajectories.png**
- **Contenido**: Visualización 3D del mapa de puntos LiDAR con ambas trayectorias superpuestas
- **Vista**: Perspectiva isométrica
- **Información**: Muestra la densidad del mapa y la cobertura espacial

### 3. **rtk_trajectory_2d.png**
- **Contenido**: Trayectoria pura RTK-GPS en 2D (vista de pájaro)
- **Propósito**: Ground truth de referencia
- **Detalles**: Coordenadas UTM con marcadores cada N frames

### 4. **error_analysis.png**
- **Contenido**: Gráficas de error temporal
  - Error de posición (cm) vs tiempo
  - Error de orientación (°) vs tiempo
  - Histograma de distribución de errores
- **Métricas**: RMS, máximo, desviación estándar

### 5. **drift_correction.png**
- **Contenido**: Visualización de eventos de corrección de deriva
- **Información**:
  - Puntos de detección de deriva
  - Magnitud de corrección aplicada
  - Línea temporal de deriva acumulada

### 6. **performance_metrics.png**
- **Contenido**: Dashboard de métricas de rendimiento
  - Tiempo de procesamiento por frame
  - Uso de memoria
  - Tasa de frames procesados
  - Comparación con estado del arte

## 🖼️ Generar las Figuras

### Método Automático

Las figuras se generan automáticamente al ejecutar el script principal:

```matlab
% Ejecutar el procesamiento completo
lidar_slam_3d_rtk_professional_v_clusters_mejorado();

% Las figuras se guardan automáticamente en:
% results/figures/*.png
```

### Método Manual

Si necesitas regenerar figuras específicas:

```matlab
% 1. Cargar resultados procesados
load('results/slam_results.mat');

% 2. Generar figuras individuales
figure('Position', [100, 100, 1200, 800]);

% Comparación de trayectorias
subplot(2,2,1);
plot(trajectory_v1(:,1), trajectory_v1(:,2), 'r-', 'LineWidth', 2);
hold on;
plot(trajectory_v2(:,1), trajectory_v2(:,2), 'b-', 'LineWidth', 2);
plot(rtk_trajectory(:,1), rtk_trajectory(:,2), 'g--', 'LineWidth', 1.5);
legend('Vuelta 1', 'Vuelta 2', 'RTK GT');
title('Comparación de Trayectorias');
xlabel('Este (m)'); ylabel('Norte (m)');
grid on; axis equal;

% 3. Guardar con alta resolución
saveas(gcf, 'results/figures/trajectory_comparison.png');
print(gcf, 'results/figures/trajectory_comparison_highres.png', '-dpng', '-r300');
```

## 📐 Especificaciones de Exportación

### Configuración Recomendada

```matlab
% Tamaño de figura
fig = figure('Position', [100, 100, 1920, 1080]);  % Full HD
set(fig, 'Color', 'white');

% Fuentes legibles
set(gca, 'FontSize', 12);
set(gca, 'FontName', 'Arial');

% Exportación de alta calidad
exportgraphics(gcf, 'output.png', 'Resolution', 300);  % 300 DPI

% O usando print:
print(gcf, 'output.png', '-dpng', '-r300');
```

### Resoluciones

```
Para README:        1200 x 800 px  (suficiente)
Para presentación:  1920 x 1080 px (Full HD)
Para paper:         300 DPI mínimo
```

## 🎨 Paleta de Colores Consistente

Para mantener coherencia visual con el README:

```matlab
% Colores MATLAB profesionales
color_v1 = [0.85, 0.33, 0.10];  % Naranja
color_v2 = [0.00, 0.45, 0.74];  % Azul MATLAB
color_rtk = [0.47, 0.67, 0.19]; % Verde
color_error = [0.93, 0.69, 0.13]; % Dorado

% Aplicar:
plot(x, y, 'Color', color_v1, 'LineWidth', 2);
```

## 📊 Checklist de Figuras

Verifica que todas las figuras incluyan:

- [ ] Título descriptivo
- [ ] Etiquetas de ejes con unidades
- [ ] Leyenda (si hay múltiples series)
- [ ] Grid activado (cuando sea apropiado)
- [ ] Tamaño de fuente legible (≥12pt)
- [ ] Alta resolución (≥150 DPI)
- [ ] Fondo blanco (mejor para impresión)
- [ ] Colores contrastantes (accesibilidad)

## 🔧 Troubleshooting

### Las figuras no se generan

```matlab
% Verificar directorio
if ~exist('results/figures', 'dir')
    mkdir('results/figures');
end

% Forzar generación de figuras
set(0, 'DefaultFigureVisible', 'on');
```

### Calidad baja en exportación

```matlab
% Usar exportgraphics en lugar de saveas
exportgraphics(gcf, 'output.png', 'Resolution', 300);
```

### Figuras en blanco

```matlab
% Asegurarse de que la figura está dibujada
drawnow;
pause(0.5);  % Pequeña pausa antes de guardar
saveas(gcf, 'output.png');
```

---

## 🚀 Scripts de Generación Disponibles

Estos scripts ya existen en el repositorio para generar visualizaciones:

```matlab
generar_imagen_mapeo_v1_profesional.m
generar_imagenes_para_readme.m
visualizar_pipeline_slam_4_etapas.m
analyze_trajectory_characteristics.m
```

Ejecuta cualquiera de ellos para generar figuras profesionales.

---

<div align="center">

**Nota**: Las figuras se generan automáticamente durante el procesamiento principal.

</div>
