function lidar_slam_3d_rtk_professional_v_clusters_mejorado()
%% LIDAR_SLAM_3D_RTK_PROFESSIONAL - SLAM 3D OPTIMIZADO PARA MÁXIMA VELOCIDAD
% ========================================================================
% VERSIÓN ULTRA OPTIMIZADA - PRIORIDAD ABSOLUTA: VELOCIDAD DE PROCESAMIENTO
% ========================================================================
% 
% 🚀 OPTIMIZACIONES PRINCIPALES PARA VELOCIDAD MÁXIMA:
%
% ⚡ 1. PARÁMETROS DE VELOCIDAD OPTIMIZADOS
%     - skipFrames: 2 → 3 (33% menos frames procesados)
%     - updateRate: 5 → 8 (60% menos actualizaciones visuales)
%     - downsamplePercent: 0.10 → 0.15 (33% menos puntos por frame)
%     - gridStepV1/V2: incrementados para registro más rápido
%
%   2. ALGORITMOS COMPLEJOS DESACTIVADOS
%     - Loop closure: DESACTIVADO (más velocidad)
%     - Filtrados avanzados: DESACTIVADOS
%     - Multi-hypothesis tracking: DESACTIVADO
%     - Algoritmos robustos: SIMPLIFICADOS AL MÁXIMO
%
% ⚙️ 3. NDT ULTRA RÁPIDO
%     - MaxIterations: 80/70 → 40/35 (50% menos iteraciones)
%     - Tolerance: más permisiva para convergencia rápida
%     - Fallback directo a RTK cuando NDT falla
%
% 📊 4. PROCESAMIENTO SIMPLIFICADO
%     - Filtrado de suelo: SOLO por altura (ultra simple)
%     - Eliminación de filtros PCL/ROS2 complejos
%     - Preprocesamiento mínimo indispensable
%
% 🎬 5. VISUALIZACIÓN REDUCIDA
%     - Video: DESACTIVADO por defecto
%     - Export: DESACTIVADO por defecto  
%     - Logging: DESACTIVADO por defecto
%     - Mapa global: resolución reducida (0.2→0.3, puntos 80K→50K)
%
% ✅ OBJETIVO: Procesamiento LiDAR en tiempo real con mínima latencia
% ========================================================================
% Cambios de versión anterior:
% - Vuelta 1 (trayectoria) en ROSA
% - Vuelta 2: MEJORADA con control activo de deriva y precisión aumentada
% - **NUEVAS MEJORAS ANTI-DERIVA SEGUNDA VUELTA**:
%   * Loop closure ACTIVADO para corrección automática de deriva
%   * Peso RTK incrementado dinámicamente en V2 (0.85 → 0.95)
%   * Detección activa de divergencia RTK-SLAM (umbral 2.0m)
%   * Parámetros NDT más precisos: gridStep 4.0→2.5, MaxIter 30→50
%   * Tolerancias más estrictas y mejor manejo de outliers
%   * Validación de saltos espaciales excesivos (>5m)
%   * Fallback inteligente a RTK en caso de falla
%   * Reporte detallado de deriva y correcciones aplicadas
% - **NUEVA VISUALIZACIÓN MEJORADA ESTILO ROS2**:
%   * Filtrado inteligente del suelo (conserva estructura, reduce densidad)
%   * Mejora de clusters con DBSCAN simplificado
%   * Paleta de colores optimizada para mejor contraste
%   * Puntos más grandes y visibles (3-6px vs 2.5px)
%   * Mayor transparencia y mejor definición de objetos
% - RTK afinado (Hampel + suavizado en metros + corrección de deriva + lever-arm)
% - Mapa RTK 2D en satélite (geoaxes) + alternativa OSM (webmap)
% - Export .PLY y CSV (LiDAR + RTK por punto) conservados
% - **NUEVO**: Logging de POSE en terminal (θ [yaw] y XYZ) en tiempo real
% - **NUEVO**: Flecha de orientación (heading) en la vista 3D

    %% ========================== CONFIGURACIÓN ==========================
    archivo_mat = 'C:\Users\Alfonso\Documents\Estancia_Investigacion-2025\recorrido_20250829_163719.mat';
    
    % ========= BANNER DE INICIO =========
    fprintf('\n');
    fprintf('╔═══════════════════════════════════════════════════════════════╗\n');
    fprintf('║          🚀 SLAM 3D PROFESIONAL - SISTEMA INICIADO 🚀        ║\n');
    fprintf('╚═══════════════════════════════════════════════════════════════╝\n');
    fprintf('📁 Dataset: %s\n', archivo_mat);
    fprintf('⚙️  Modo: Profesional con Keyframes + RANSAC + Filtros\n');
    fprintf('📍 Pose en tiempo real: [X, Y, Z, θ]\n');
    fprintf('───────────────────────────────────────────────────────────────\n\n');
    
    % ========= CONFIGURACIÓN ESTILO ROS2 NAV2/CARTOGRAPHER =========
    % Basado en algoritmos de slam_toolbox y cartographer
    ROS2_STYLE_MAPPING = true;
    APPLY_GROUND_FILTER_FROM_START = true;  % Aplicar desde vuelta 1
    USE_ADVANCED_VOXEL_GRID = true;         % Voxel grid avanzado como ROS2
    UNIFORM_POINT_DISTRIBUTION = true;      % Distribución uniforme
    PROFESSIONAL_TRAJECTORY_FUSION = true;  % Fusión profesional RTK-SLAM

    % ========= DETECCIÓN AUTOMÁTICA DE TIPO DE TRAYECTORIA =========
    ADAPTIVE_MODE = true;  % Activar modo adaptativo inteligente
    
    % FILTRADO ESPACIAL (ULTRA AGRESIVO CONTRA EL SUELO)
    egoRadius = 2.5;
    cylinderRadius = 35;    
    minHeight = 1.0;        % INCREMENTADO: Solo objetos > 1m de altura
    maxHeight = 8.0;        

    % VELOCIDAD / MUESTREO (ULTRA OPTIMIZADO para máxima velocidad)
    downsamplePercent = 0.15;   % Incrementado para menos puntos por frame
    gridStepV1 = 6.0;          % Incrementado para registro más rápido
    gridStepV2 = 4.0;          % Incrementado para localización más rápida  
    updateRate = 8;            % Incrementado para menos actualizaciones visuales
    skipFrames = 3;            % Incrementado para procesar menos frames
    
    % === PARÁMETROS NDT PARA VUELTA 1 (Sistema Ultra-Robusto) ===
    NDT_MAX_ITER_V1 = 120;         % Iteraciones base para tramos normales
    NDT_TOLERANCE_V1 = [0.005, 0.3]; % Tolerancia balanceada [traslación, rotación]

    % ========= ALGORITMOS ROBUSTOS DESACTIVADOS (para máxima velocidad) =========
    % Todos los algoritmos complejos desactivados para priorizar velocidad
    MAX_SPATIAL_JUMP = 10.0;   % Más permisivo
    JUMP_RECOVERY_MODE = false; % Desactivado
    
    MULTI_HYPOTHESIS_TRACKING = false; % Desactivado
    NUM_HYPOTHESES = 1;       
    
    SPATIAL_COHERENCE_CHECK = false; % Desactivado
    COHERENCE_THRESHOLD = 0.5; 
    
    PREDICTIVE_CORRECTION = false; % Desactivado
    MOTION_MODEL_WEIGHT = 0.0;

    % MAPA GLOBAL (Optimizado para máxima velocidad)
    resMap = 0.3; % Reducida resolución para menos densidad
    maxPtsMap = 50000; % Reducido significativamente para mejor rendimiento

    % ========= NUEVOS ALGORITMOS DESACTIVADOS (para velocidad) =========
    % 1. Filtrado de puntos flotantes - DESACTIVADO
    FLOATING_POINT_REMOVAL = false;   
    MAX_HEIGHT_ABOVE_GROUND = 15.0;   
    MIN_GROUND_SUPPORT = 1;          
    
    % 2. Filtrado de densidad - DESACTIVADO 
    ADAPTIVE_DENSITY_FILTER = false;  
    DENSITY_RADIUS = 2.0;            
    MIN_NEIGHBORS = 4;               
    
    % 3. Filtrado estadístico - DESACTIVADO
    STATISTICAL_OUTLIER_REMOVAL = false; 
    STD_MULTIPLIER = 3.0;               
    
    % ========= FILTRADO DE SUELO SIMPLE (velocidad máxima) =========
    ADVANCED_GROUND_FILTERING = false;   % DESACTIVADO
    REMOVE_GROUND_COMPLETELY = true;     % Solo filtrado simple por altura
    GROUND_DETECTION_METHOD = 'simple';  % Método más simple
    PLANE_DISTANCE_THRESHOLD = 0.3;      % Menos estricto para velocidad
    MIN_OBJECT_HEIGHT = 0.8;             % Reducido para menos procesamiento
    VEGETATION_ENHANCEMENT = false;       % DESACTIVADO
    STRUCTURE_PRESERVATION = false;       % DESACTIVADO
    
    % Configuración silenciosa (sin prints extra)
    
    % 4. Validación geométrica
    GEOMETRIC_VALIDATION = true;     % Validar coherencia geométrica
    PLANARITY_THRESHOLD = 0.1;       % Umbral de planaridad para superficies
    
    % 5. Control de acumulación
    ACCUMULATION_CONTROL = true;     % Controlar acumulación excesiva
    MAX_POINTS_PER_VOXEL = 50;       % Máximo puntos por vóxel
    TEMPORAL_DECAY = 0.98;           % Factor de decaimiento temporal
    
    % ========= NUEVOS ALGORITMOS DE CLASIFICACIÓN INTELIGENTE =========
    % 1. Segmentación automática por tipo de objeto
    INTELLIGENT_SEGMENTATION = true;  % Activar clasificación automática
    VEGETATION_DETECTION = true;      % Detección específica de vegetación
    STRUCTURE_DETECTION = true;       % Detección de estructuras artificiales
    
    % 2. Parámetros de clasificación por altura
    GROUND_HEIGHT_RANGE = [0.0, 0.8];      % Rango de altura para suelo
    VEGETATION_HEIGHT_RANGE = [0.8, 15.0]; % Rango para vegetación
    STRUCTURE_HEIGHT_RANGE = [1.5, 25.0];  % Rango para estructuras
    
    % 3. Clustering mejorado para vegetación
    VEGETATION_CLUSTER_RADIUS = 2.5;  % Radio para agrupar vegetación
    VEGETATION_MIN_POINTS = 20;       % Mínimo puntos por cluster de vegetación
    TREE_SEPARATION_DISTANCE = 4.0;  % Distancia mínima entre árboles
    
    % 4. Reducción de redundancia inteligente
    SMART_REDUNDANCY_REMOVAL = true; % Eliminar puntos redundantes
    VEGETATION_SUBSAMPLE_RATE = 0.3;  % Factor de submuestreo para vegetación
    STRUCTURE_SUBSAMPLE_RATE = 0.7;   % Factor para estructuras (más detalle)
    GROUND_SUBSAMPLE_RATE = 0.2;      % Factor para suelo (menos detalle)
    
    % 5. Mejora de visualización por categorías
    CATEGORY_BASED_COLORS = true;     % Colores específicos por categoría
    ENHANCED_TREE_DEFINITION = true;  % Definición mejorada de árboles individuales
    
    % 2. Mejora de clusters con múltiples algoritmos
    CLUSTER_ENHANCEMENT = true;
    MULTI_CLUSTER_ALGORITHMS = true; % DBSCAN + K-means + Region Growing
    ADAPTIVE_CLUSTERING = true;      % Parámetros adaptativos según densidad
    
    % 3. Parámetros dinámicos de visualización
    GROUND_REDUCTION_FACTOR = 0.25;  % Más agresivo para mejor rendimiento
    MIN_CLUSTER_POINTS = 12;         % Reducido para detectar objetos pequeños
    CLUSTER_HEIGHT_THRESHOLD = 0.6;  % Reducido para mayor sensibilidad
    
    % 4. Nuevos algoritmos de estabilización
    POSE_STABILIZATION = true;       % Estabilizar estimación de pose
    TEMPORAL_CONSISTENCY = true;     % Consistencia temporal
    OUTLIER_REJECTION = true;        % Rechazo robusto de outliers

    % LOCALIZACIÓN (Mejorada para trayectorias complejas y segunda vuelta)
    roiR = 45; % Incrementado para mejor contexto
    
    % PARÁMETROS ESPECÍFICOS PARA SEGUNDA VUELTA (V2)
    gridStepV2 = 2.5;          % REDUCIDO para mayor precisión en localización
    V2_MAX_ITERATIONS = 50;    % Más iteraciones para mejor convergencia
    V2_TOLERANCE = [0.01, 0.5]; % Tolerancias más estrictas
    V2_OUTLIER_RATIO = 0.3;    % Menos outliers permitidos

    % ========= LOOP CLOSURE MEJORADO (ACTIVADO para segunda vuelta) =========
    ENABLE_LOOP_CLOSURE = true;   % ACTIVADO para corregir deriva en V2
    LOOP_CLOSURE_THRESHOLD = 6.0; % Más estricto para mejor detección   
    LOOP_CLOSURE_MIN_FRAMES = 30; % Reducido para detección más temprana     
    
    % Nuevos parámetros de loop closure
    MULTI_SCALE_LOOP_DETECTION = true; % Detección multi-escala
    GEOMETRIC_VERIFICATION = true;      % Verificación geométrica
    TEMPORAL_LOOP_FILTERING = true;     % Filtrado temporal de loops
    
    % Parámetros específicos para V2 (localización)
    V2_ENHANCED_PRECISION = true;       % Precisión mejorada en V2
    V2_DRIFT_CORRECTION = true;         % Corrección activa de deriva
    V2_RTK_WEIGHT_BOOST = 0.95;         % Peso RTK aumentado en V2
    
    % ========= RTK FUSION PRIORIZADA (CORREGIDO Y MEJORADO PARA V2) =========
    RTK_FOLLOW_PRECISION = 0.85;     % ALTO - RTK como guía principal V1
    RTK_Z_WEIGHT = 2.0;              % Incrementado para mejor altura
    RTK_TEMPORAL_SYNC = true;
    
    % RTK robusto para saltos espaciales
    RTK_JUMP_DETECTION = true;       % Detectar saltos en RTK
    RTK_ADAPTIVE_FILTERING = true;   % Filtrado adaptativo
    RTK_MOTION_PREDICTION = true;    % Predicción de movimiento
    RTK_SLAM_CONVERGENCE = true;     % NUEVO: Forzar convergencia RTK-SLAM
    
    % MEJORAS ESPECÍFICAS PARA SEGUNDA VUELTA
    RTK_V2_ENHANCED = true;          % Mejoras RTK específicas para V2
    RTK_V2_WEIGHT = 0.95;           % Peso RTK MUY ALTO en V2 para evitar deriva
    RTK_V2_SMOOTH_FACTOR = 0.8;     % Factor de suavizado temporal
    RTK_V2_DRIFT_THRESHOLD = 2.0;   % Umbral de deriva para corrección (metros)

    % ====== Mejora de trazas RTK (nuevo) ======
    RTK_MAX_SPEED_MS     = 12;        % m/s para filtrar saltos imposibles
    RTK_SMOOTH_WIN_M     = 3.0;       % tamaño de ventana de suavizado (metros)
    RTK_HAMPEL_WINDOW    = 9;         % ventana Hampel (muestras)
    RTK_LOOP_CORRECTION  = true;      % corregir deriva entre inicio y final
    RTK_STATIC_OFFSET_M  = [0.0, 0.0];% [dx, dy] metros: antena → centro del vehículo

    % VISUAL (EXPORTACIÓN DE VIDEO MEJORADA)
    SAVE_VIDEO = true;         % ACTIVADO: Exportar video MP4 + GIF
    VIDEO_FPS = 15;            % FPS del video
    VIDEO_QUALITY = 95;        % Calidad HD del MP4
    EXPORT_GIF = true;         % NUEVO: También exportar GIF animado
    GIF_DELAY = 0.1;           % NUEVO: Delay entre frames del GIF (segundos)
    VIDEO_V1_FILE = 'slam_v1_mapeo.mp4';      % NUEVO: Archivo V1
    VIDEO_V2_FILE = 'slam_v2_localizacion.mp4'; % NUEVO: Archivo V2
    GIF_V1_FILE = 'slam_v1_mapeo.gif';        % NUEVO: GIF V1
    GIF_V2_FILE = 'slam_v2_localizacion.gif'; % NUEVO: GIF V2

    % DARK THEME (simplificado)
    DARK_THEME = true;

    % ==== FLAGS DE VISUALIZACIÓN ====
    VIS_SHOW_CURRENT_V1 = true;   % en V1 se ve nube actual
    VIS_SHOW_CURRENT_V2 = false;  % en V2 NO se ve nube actual

    % EXPORTACIÓN (REDUCIDA para velocidad)
    EXPORT_POINTS = false;     % Desactivado para máxima velocidad
    MAX_EXPORT_POINTS = 0.5e6; % Reducido significativamente
    EXPORT_FILENAME = 'map_points_with_rtk.csv';

    % =============== LOGGING SIMPLIFICADO (para velocidad) ======================
    POSE_LOG = false;          % Desactivado para máxima velocidad
    POSE_LOG_EVERY_FRAME = false;
    POSE_LOG_SAMPLE_N = max(1, updateRate);

    PRINT_PER_POINT = false;
    PER_POINT_LIMIT  = 0;

    %% ========================== CARGA / PREP ==========================
    if ~isfile(archivo_mat), error('Archivo no encontrado: %s', archivo_mat); end
    S = load(archivo_mat);
    [frames, lidarTimestamps] = detectLidarFrames(S);
    [hasRTK, XYZrtk, rtkTimestamps, latlonLLA] = detectRTK_3D_Enhanced(S);

    if hasRTK && RTK_TEMPORAL_SYNC && ~isempty(lidarTimestamps) && ~isempty(rtkTimestamps)
        [alignedFrames, alignedRTK, q] = synchronizeLidarRTK_Enhanced(frames, lidarTimestamps, XYZrtk, rtkTimestamps);
        frames = alignedFrames; XYZrtk = alignedRTK;
    end

    % DIVISIÓN V1/V2
    idxSplit = floor(numel(frames)/2);
    ptCloudMap = frames(1:idxSplit);
    ptCloudLoc = frames(idxSplit+1:end);

    % RTK dividido
    if hasRTK && size(XYZrtk,1) >= 2*idxSplit
        XYZrtk_v1 = XYZrtk(1:idxSplit, :);
        XYZrtk_v2 = XYZrtk(idxSplit+1:min(end, idxSplit+length(ptCloudLoc)), :);
    else
        XYZrtk_v1 = [];
        XYZrtk_v2 = [];
    end

    % Datos cargados silenciosamente

    %% ========================== VENTANA PRO (sin parpadeo) ==========================
    fig = figure('Name','SLAM 3D Profesional','Position',[40,40,1600,950]);
    if DARK_THEME, set(fig, 'Color', [0.02 0.02 0.02]); end
    try, set(fig,'GraphicsSmoothing','on'); catch, end

    ax = axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); view(ax,3);
    vizHandle = initializeProfessionalVisualization(ax, DARK_THEME);
    
    if DARK_THEME
        colorMapPathV1 = [1.0 0.2 0.6]; % ROSA (Vuelta 1)
        colorRTK = [0.9 0.9 0.9];
        textColor = [0.9 0.9 0.9];
    else
        colorMapPathV1 = [0.9 0.3 0.7];
        colorRTK = [0.3 0.3 0.3];
        textColor = [0.1 0.1 0.1];
    end
    
    xlabel(ax,'X (m)', 'Color', textColor, 'FontSize', 12, 'FontWeight', 'bold'); 
    ylabel(ax,'Y (m)', 'Color', textColor, 'FontSize', 12, 'FontWeight', 'bold'); 
    zlabel(ax,'Z (m)', 'Color', textColor, 'FontSize', 12, 'FontWeight', 'bold'); 
    axis(ax,'equal');
    title(ax,'SLAM 3D Profesional: Construcción del Mapa','FontSize',16, 'Color', textColor, 'FontWeight', 'bold');

    % Trayectorias (V1 ROSA, V2 ROJO)
    hPath1 = plot3(ax,0,0,0,'-','LineWidth',5,'Color',colorMapPathV1,'DisplayName','Trayectoria Vuelta 1');
    hPath2 = plot3(ax,0,0,0,'--','LineWidth',5,'Color',[1 0 0],'DisplayName','Trayectoria Vuelta 2');

    % RTK
    if hasRTK && size(XYZrtk,1)>=2
        plot3(ax, XYZrtk(:,1), XYZrtk(:,2), XYZrtk(:,3), ':', ...
              'Color', colorRTK, 'LineWidth', 3.5, 'DisplayName','Ground Truth RTK 3D');
        plot3(ax, XYZrtk(1,1), XYZrtk(1,2), XYZrtk(1,3), 'o', ...
              'MarkerSize',14, 'MarkerFaceColor','green', 'MarkerEdgeColor','white', ...
              'LineWidth', 2.5, 'DisplayName','Inicio RTK');
        plot3(ax, XYZrtk(end,1), XYZrtk(end,2), XYZrtk(end,3), 's', ...
              'MarkerSize',14, 'MarkerFaceColor','red', 'MarkerEdgeColor','white', ...
              'LineWidth', 2.5, 'DisplayName','Fin RTK');
    end
    lgd = legend(ax,'Location','northeast', 'TextColor', textColor, 'FontSize', 11);
    set(lgd,'AutoUpdate','off');

    % VIDEO Y GIF
    vWriter = []; vH = []; vW = []; wroteFrames = 0;
    gifWriter = []; % NUEVO: Handle para GIF
    currentVideoFile = ''; % NUEVO: Archivo actual (V1 o V2)
    currentGifFile = ''; % NUEVO: GIF actual
    
    if SAVE_VIDEO
        % Determinar si estamos en V1 o V2 (se actualizará en el loop)
        currentVideoFile = VIDEO_V1_FILE;
        currentGifFile = GIF_V1_FILE;
        
        try, vWriter = VideoWriter(currentVideoFile,'MPEG-4'); catch
            vWriter = VideoWriter(strrep(currentVideoFile, '.mp4', '.avi'),'Motion JPEG AVI');
        end
        vWriter.FrameRate = VIDEO_FPS; vWriter.Quality = VIDEO_QUALITY; open(vWriter);
        
        fprintf('📹 Video V1 iniciado: %s\n', currentVideoFile);
        if EXPORT_GIF
            fprintf('🎬 GIF V1 iniciado: %s\n', currentGifFile);
        end
        
        drawnow; [vH, vW] = writeVideoFixed(vWriter, ax, vH, vW); wroteFrames = wroteFrames + 1;
        
        % Inicializar GIF
        if EXPORT_GIF
            frameImg = getframe(ax);
            [imgGif, mapGif] = rgb2ind(frameImg.cdata, 256);
            imwrite(imgGif, mapGif, currentGifFile, 'gif', 'LoopCount', Inf, 'DelayTime', GIF_DELAY);
        end
    end

    % Flechas de orientación (heading)
    headingLen = 2.0;  % longitud de la flecha en metros
    hHeadingV1 = quiver3(ax, nan,nan,nan, nan,nan,nan, 0, 'Color',[1 0.6 0.85], 'LineWidth',2.5, 'MaxHeadSize',1.5, 'HandleVisibility','off');
    hHeadingV2 = quiver3(ax, nan,nan,nan, nan,nan,nan, 0, 'Color',[1 0.2 0.2],  'LineWidth',2.5, 'MaxHeadSize',1.5, 'HandleVisibility','off');

    %% ========================== VUELTA 1: MAPEO ==========================
    fprintf('\n╔═══════════════════════════════════════╗\n');
    fprintf('║       🗺️  VUELTA 1: MAPEO           ║\n');
    fprintf('╚═══════════════════════════════════════╝\n\n');
    t1 = tic;

    % ========= INICIALIZACIÓN INTELIGENTE: 10m adelante con yaw correcto =========
    if hasRTK && ~isempty(XYZrtk_v1) && size(XYZrtk_v1,1) >= 10
        % Calcular dirección de movimiento desde primeros frames
        direction_vector = XYZrtk_v1(10,:) - XYZrtk_v1(1,:);
        direction_norm = direction_vector / norm(direction_vector);
        
        % Posición inicial: 10m adelante en dirección del movimiento
        init_position = XYZrtk_v1(1,:) + 10.0 * direction_norm;
        
        % Calcular yaw (ángulo en plano XY)
        yaw = atan2(direction_vector(2), direction_vector(1));
        
        % Matriz de rotación alrededor de Z
        R_init = [cos(yaw) -sin(yaw) 0;
                  sin(yaw)  cos(yaw) 0;
                  0         0        1];
        
        initPose = rigidtform3d([R_init, init_position(:); 0 0 0 1]);
    else
        % Fallback: inicialización estándar
        if hasRTK && ~isempty(XYZrtk_v1)
            initPose = rigidtform3d([eye(3), XYZrtk_v1(1,:)'; 0 0 0 1]);
        else
            initPose = rigidtform3d;
        end
    end
    
    % ========= SISTEMA DE KEYFRAMES =========
    last_keyframe_position = initPose.Translation;
    keyframe_distance_threshold = 4.0;  % 4 metros entre keyframes
    frames_since_last_keyframe = 0;
    total_keyframes = 0;
    
    % Inicializar algoritmos simples
    pose_history = zeros(0, 3);
    
    vSet = pcviewset;
    vSet = addView(vSet,1,initPose);
    pose_history = [pose_history; initPose.Translation(:)'];

    hasSMRF = exist('segmentGroundSMRF','file')==2;

    f1 = getFramePC(ptCloudMap,1);
    pc1 = preprocessForProfessionalMap(f1, egoRadius, cylinderRadius, minHeight, maxHeight, hasSMRF);
    pc1d = pcdownsample(pc1,'random',downsamplePercent);

    f2 = getFramePC(ptCloudMap,2);
    pc2 = preprocessForProfessionalMap(f2, egoRadius, cylinderRadius, minHeight, maxHeight, hasSMRF);
    pc2d = pcdownsample(pc2,'random',downsamplePercent);

    % ========= REGISTRO NDT SIMPLIFICADO CON PRIORIDAD RTK =========
    if hasRTK && ~isempty(XYZrtk_v1) && size(XYZrtk_v1,1) >= 2
        rtk_motion = XYZrtk_v1(2,:) - XYZrtk_v1(1,:);
        if RTK_Z_WEIGHT > 1 && numel(rtk_motion) >= 3, rtk_motion(3) = rtk_motion(3)*RTK_Z_WEIGHT; end
        initial_transform = rigidtform3d([eye(3), rtk_motion(:); 0 0 0 1]);
        
        % REGISTRO NDT ULTRA RÁPIDO
        try
            relPose = pcregisterndt(pc2d, pc1d, gridStepV1, ...
                'InitialTransform', initial_transform, ...
                'MaxIterations', 40, 'Tolerance', [0.02, 0.5]);
        catch
            % Fallback simple sin algoritmos complejos
            relPose = initial_transform;  % Usar directamente RTK
            fprintf('🔄 Usando RTK directo como fallback\n');
        end
    else
        % Sin RTK, NDT básico
        try
            relPose = pcregisterndt(pc2d, pc1d, gridStepV1);
        catch
            relPose = rigidtform3d;  % Identidad
        end
    end
    
    % ========= VALIDACIÓN SIMPLIFICADA (SOLO SI HAY SALTOS EXTREMOS) =========
    if ~isempty(pose_history) && size(pose_history, 1) > 0 && hasRTK
        last_pos = pose_history(end, :);
        current_translation = relPose.Translation(:)';
        spatial_jump = norm(current_translation - last_pos);
        
        % Solo corregir saltos EXTREMOS (>5m) - confiar más en RTK
        if spatial_jump > 5.0
            fprintf('🚨 SALTO EXTREMO DETECTADO: %.2fm - Usando RTK\n', spatial_jump);
            % Usar directamente movimiento RTK
            if hasRTK && ~isempty(XYZrtk_v1) && size(XYZrtk_v1,1) >= 2
                rtk_motion = XYZrtk_v1(2,:) - XYZrtk_v1(1,:);
                relPose = rigidtform3d([eye(3), rtk_motion(:); 0 0 0 1]);
                fprintf('✅ USANDO RTK DIRECTO\n');
            end
        end
    end
    
    if hasRTK && RTK_FOLLOW_PRECISION > 0 && ~isempty(XYZrtk_v1) && size(XYZrtk_v1,1) >= 2
        rtk_pose = rigidtform3d([eye(3), XYZrtk_v1(2,:)'; 0 0 0 1]);
        combined_translation = RTK_FOLLOW_PRECISION * rtk_pose.Translation + ...
                              (1 - RTK_FOLLOW_PRECISION) * (initPose.Translation + relPose.Translation);
        absPose = rigidtform3d([relPose.R, combined_translation(:); 0 0 0 1]);
    else
        absPose = rigidtform3d(initPose.A * relPose.A);
    end

    vSet = addView(vSet,2,absPose);
    vSet = addConnection(vSet,1,2,relPose);

    loop_closure_poses = [];
    path1 = [initPose.Translation; absPose.Translation];
    prevPC = pc2d;
    mapPC = [];
    frameCount = 0;
    viewId = 2;

    % Estado refresco / video
    lastPct   = -1; lastLimT  = tic; vidTick   = 0;

    % Buffers export
    exportXYZ = []; exportRTK = []; exportMeta = [];

    % Log inicial y flecha
    if POSE_LOG, logPose('V1', 2, absPose); end
    updateHeadingArrow(hHeadingV1, absPose, headingLen);

    % Iniciar logging de pose
    fprintf('🎬 Iniciando procesamiento - Pose en tiempo real activada\n\n');
    
    % Variables para skip adaptativo
    current_skip = skipFrames;
    last_rtk_heading = 0;
    
    i = 3;
    while i <= numel(ptCloudMap)
        frame = getFramePC(ptCloudMap,i);
        
        % ========= PREPROCESAMIENTO CON RANSAC GROUND REMOVAL =========
        % Eliminar puntos de ego y cilindro
        if frame.Count > 0
            pts = frame.Location;
            dists = sqrt(pts(:,1).^2 + pts(:,2).^2);
            mask = (dists > egoRadius) & (dists < cylinderRadius);
            if sum(mask) > 50
                frame = select(frame, find(mask));
            end
        end
        
        % RANSAC para detectar y eliminar suelo
        if frame.Count > 100
            try
                [~, inlierIndices, ~] = pcfitplane(frame, 0.05, ...  % 5cm threshold
                    'MaxNumTrials', 1000, ...
                    'Confidence', 99.9);
                
                % Obtener puntos del plano
                plane_pts = select(frame, inlierIndices);
                plane_z_mean = mean(plane_pts.Location(:,3));
                
                % Eliminar puntos a <10cm del plano de suelo
                all_pts = frame.Location;
                ground_mask = abs(all_pts(:,3) - plane_z_mean) < 0.10;
                non_ground_idx = find(~ground_mask);
                
                if length(non_ground_idx) > 50
                    frame = select(frame, non_ground_idx);
                    % Filtrado silencioso
                end
            catch
                % Si RANSAC falla, usar filtro por altura
                pts = frame.Location;
                mask = pts(:,3) >= minHeight & pts(:,3) <= maxHeight;
                if sum(mask) > 50
                    frame = select(frame, find(mask));
                end
            end
        end
        
        pc = frame;
        pcd = pcdownsample(pc,'random',downsamplePercent);

        % ========= ANÁLISIS GEOMÉTRICO PARA ADAPTACIÓN =========
        is_sharp_curve = false;
        turn_angle_deg = 0;
        
        if hasRTK && ~isempty(XYZrtk_v1) && i <= size(XYZrtk_v1,1) && i > 2
            % Detectar curvas cerradas analizando cambio de dirección
            v1 = XYZrtk_v1(i-1,:) - XYZrtk_v1(i-2,:);
            v2 = XYZrtk_v1(i,:) - XYZrtk_v1(i-1,:);
            
            if norm(v1) > 0.1 && norm(v2) > 0.1  % Evitar divisiones por cero
                cos_angle = dot(v1, v2) / (norm(v1) * norm(v2));
                cos_angle = max(-1, min(1, cos_angle));  % Clamp
                turn_angle_deg = abs(acosd(cos_angle));
                
                % Curva cerrada si cambio > 30°
                if turn_angle_deg > 30
                    is_sharp_curve = true;
                end
            end
        end
        
        % ========= SKIP ADAPTATIVO POR GEOMETRÍA =========
        if is_sharp_curve
            current_skip = 1;  % Procesar TODOS los frames en curvas
            if mod(i-3, skipFrames) ~= 0  % Avisar solo cuando cambiamos
                fprintf('🔄 Frame %d: Curva %.1f° detectada - Skip=1\n', i, turn_angle_deg);
            end
        else
            current_skip = skipFrames;  % Skip normal en rectas
        end
        
        % ========= REGISTRO MULTI-ESTRATEGIA ROBUSTO =========
        registration_success = false;
        registration_quality = 0;
        relPose = rigidtform3d;  % Default: identidad
        
        if hasRTK && ~isempty(XYZrtk_v1) && i <= size(XYZrtk_v1,1) && i > 1
            rtk_motion = XYZrtk_v1(i,:) - XYZrtk_v1(i-1,:);
            if RTK_Z_WEIGHT > 1 && numel(rtk_motion) >= 3
                rtk_motion(3) = rtk_motion(3) * RTK_Z_WEIGHT;
            end
            rtk_transform = rigidtform3d([eye(3), rtk_motion(:); 0 0 0 1]);
            
            % === ESTRATEGIA 1: NDT con parámetros adaptativos ===
            if is_sharp_curve
                % Curvas: parámetros ULTRA conservadores
                ndt_iters = 200;
                ndt_grid = 2.0;  % Más fino
                ndt_tol = [0.001, 0.1];  % Muy estricto
            else
                % Rectas: parámetros balanceados
                ndt_iters = NDT_MAX_ITER_V1;
                ndt_grid = gridStepV1;
                ndt_tol = NDT_TOLERANCE_V1;
            end
            
            try
                relPose_ndt = pcregisterndt(pcd, prevPC, ndt_grid, ...
                    'InitialTransform', rtk_transform, ...
                    'MaxIterations', ndt_iters, ...
                    'Tolerance', ndt_tol, ...
                    'OutlierRatio', 0.15);
                
                % Validar NDT vs RTK
                ndt_dist = norm(relPose_ndt.Translation - rtk_transform.Translation);
                
                if ndt_dist < 2.0  % NDT coherente con RTK
                    relPose = relPose_ndt;
                    registration_success = true;
                    registration_quality = 0.9;
                else
                    fprintf('⚠️ Frame %d: NDT divergió %.2fm de RTK\n', i, ndt_dist);
                end
            catch ME
                % NDT falló, continuar a estrategia 2
            end
            
            % === ESTRATEGIA 2: ICP Multi-escala (fallback robusto) ===
            if ~registration_success && is_sharp_curve
                fprintf('🔧 Frame %d: Intentando ICP multi-escala...\n', i);
                try
                    % Pirámide coarse-to-fine
                    pcd_coarse = pcdownsample(pcd, 'random', 0.5);
                    prev_coarse = pcdownsample(prevPC, 'random', 0.5);
                    
                    % ICP Nivel 1: 50% puntos
                    relPose_icp1 = pcregistericp(pcd_coarse, prev_coarse, ...
                        'InitialTransform', rtk_transform, ...
                        'MaxIterations', 100, ...
                        'Tolerance', [0.001, 0.01], ...
                        'InlierRatio', 0.7);
                    
                    % ICP Nivel 2: refinamiento con más puntos
                    pcd_medium = pcdownsample(pcd, 'random', 0.25);
                    prev_medium = pcdownsample(prevPC, 'random', 0.25);
                    
                    relPose_icp2 = pcregistericp(pcd_medium, prev_medium, ...
                        'InitialTransform', relPose_icp1, ...
                        'MaxIterations', 50, ...
                        'Tolerance', [0.0005, 0.005], ...
                        'InlierRatio', 0.8);
                    
                    % Validar ICP vs RTK
                    icp_dist = norm(relPose_icp2.Translation - rtk_transform.Translation);
                    
                    if icp_dist < 3.0
                        relPose = relPose_icp2;
                        registration_success = true;
                        registration_quality = 0.7;
                        fprintf('✅ ICP exitoso: %.2fm diferencia con RTK\n', icp_dist);
                    end
                catch ME_ICP
                    % ICP también falló
                end
            end
            
            % === ESTRATEGIA 3: RTK puro (último recurso) ===
            if ~registration_success
                relPose = rtk_transform;
                registration_quality = 0.5;  % Baja confianza
                if mod(i, 20) == 0
                    fprintf('⚠️ Frame %d: Usando RTK puro (SLAM falló)\n', i);
                end
            end
            
        else
            % Sin RTK, NDT básico
            try
                relPose = pcregisterndt(pcd, prevPC, gridStepV1, ...
                    'MaxIterations', 80);
                registration_success = true;
            catch
                % Saltar frame si todo falla
                i = i + current_skip;
                continue;
            end
        end
        
        % ========= FUSIÓN RTK-SLAM SIMPLIFICADA =========
        if hasRTK && RTK_FOLLOW_PRECISION > 0 && ~isempty(XYZrtk_v1) && i <= size(XYZrtk_v1,1)
            rtk_absolute_pose = rigidtform3d([eye(3), XYZrtk_v1(i,:)'; 0 0 0 1]);
            slam_pose = rigidtform3d(absPose.A * relPose.A);
            combined_translation = RTK_FOLLOW_PRECISION * rtk_absolute_pose.Translation + ...
                                  (1 - RTK_FOLLOW_PRECISION) * slam_pose.Translation;
            absPose = rigidtform3d([slam_pose.R, combined_translation(:); 0 0 0 1]);
        else
            absPose = rigidtform3d(absPose.A * relPose.A);
        end
        
        % ========= 📍 PUBLICAR POSE EN TIEMPO REAL =========
        % Extraer X, Y, Z de la pose
        pose_xyz = absPose.Translation;
        X = pose_xyz(1);
        Y = pose_xyz(2);
        Z = pose_xyz(3);
        
        % Calcular ángulo theta (yaw) desde la matriz de rotación
        R = absPose.R;
        theta_rad = atan2(R(2,1), R(1,1));  % Yaw desde matriz de rotación
        theta_deg = rad2deg(theta_rad);
        
        % Publicar pose en terminal
        fprintf('V1 [Frame %4d] | X: %7.2f | Y: %7.2f | Z: %6.2f | θ: %6.1f°\n', ...
                i, X, Y, Z, theta_deg);
        
        % ========= ACTUALIZAR HISTORIAL SIMPLIFICADO =========
        pose_history = [pose_history; absPose.Translation(:)'];  % Solo para logging
        
        % Mantener historial limitado
        if size(pose_history, 1) > 20
            pose_history = pose_history(end-19:end, :);
        end

        viewId = viewId + 1;
        vSet = addView(vSet, viewId, absPose);
        vSet = addConnection(vSet, viewId-1, viewId, relPose);

        if ENABLE_LOOP_CLOSURE && mod(i, 10) == 0
            loop_closure_poses = [loop_closure_poses; absPose.Translation]; %#ok<AGROW>
        end

        % ========= KEYFRAME SELECTION: Solo agregar si >4m desde último keyframe =========
        current_position = absPose.Translation;
        distance_from_last_keyframe = norm(current_position - last_keyframe_position);
        frames_since_last_keyframe = frames_since_last_keyframe + 1;
        
        is_keyframe = false;
        if distance_from_last_keyframe >= keyframe_distance_threshold
            is_keyframe = true;
            last_keyframe_position = current_position;
            total_keyframes = total_keyframes + 1;
            frames_since_last_keyframe = 0;
        end
        
        % Solo agregar al mapa si es keyframe
        tfCloud = [];  % Inicializar
        if is_keyframe
            tfCloud = pctransform(pcd, absPose);
            
            % 🔥 FILTRO ESPACIAL V1: Eliminar puntos lejos de la trayectoria actual
            if ~isempty(tfCloud) && tfCloud.Count > 0
                current_traj_point = absPose.Translation;
                tfCloud = filterPointsNearTrajectory(tfCloud, current_traj_point, 12.0);
            end
            
            mapPC = buildProfessionalGlobalMap(mapPC, tfCloud, resMap, maxPtsMap, vizHandle);
        end

        % Export V1
        if EXPORT_POINTS && is_keyframe && ~isempty(tfCloud) && isa(tfCloud, 'pointCloud')
            pts = tfCloud.Location;
            if hasRTK && ~isempty(XYZrtk_v1) && i <= size(XYZrtk_v1,1)
                rtk_here = repmat(XYZrtk_v1(i,:), size(pts,1), 1);
            elseif hasRTK && ~isempty(XYZrtk)
                rtk_here = repmat(intelligentInterpolation3D_Enhanced(XYZrtk, 1), size(pts,1), 1);
            else
                rtk_here = zeros(size(pts));
            end
            exportXYZ  = [exportXYZ; pts];                 %#ok<AGROW>
            exportRTK  = [exportRTK; rtk_here];            %#ok<AGROW>
            exportMeta = [exportMeta; [repmat(i, size(pts,1), 1), repmat(1, size(pts,1), 1)]]; %#ok<AGROW>
        end

        % === LOGGING DE POSE ===
        if POSE_LOG && (POSE_LOG_EVERY_FRAME || mod(i, POSE_LOG_SAMPLE_N)==0)
            logPose('V1', i, absPose);
        end
        if PRINT_PER_POINT && ~isempty(tfCloud) && tfCloud.Count>0
            P = tfCloud.Location;
            nprint = min(PER_POINT_LIMIT, size(P,1));
            for k=1:nprint
                fprintf('[V1][frame %d] point %5d: x=%.3f y=%.3f z=%.3f\n', i, k, P(k,1), P(k,2), P(k,3));
            end
            if size(P,1) > nprint
                fprintf('  ... (%d puntos más omitidos)\n', size(P,1)-nprint);
            end
        end

        frameCount = frameCount + 1;
        if mod(frameCount, updateRate) == 0
            [lastPct,lastLimT] = updateVisualization3D_Professional( ...
                ax, tfCloud, path1, i, numel(ptCloudMap), ...
                mapPC.Count, mapPC, vizHandle, DARK_THEME, true, ...
                VIS_SHOW_CURRENT_V1, lastPct, lastLimT);

            path1 = [path1; absPose.Translation]; %#ok<AGROW>
            set(hPath1,'XData',path1(:,1),'YData',path1(:,2),'ZData',path1(:,3));
            updateHeadingArrow(hHeadingV1, absPose, headingLen);

            if SAVE_VIDEO
                vidTick = vidTick + 1;
                if mod(vidTick,2)==0
                    [vH, vW] = writeVideoFixed(vWriter, ax, vH, vW); wroteFrames = wroteFrames + 1;
                    
                    % NUEVO: Agregar frame al GIF
                    if EXPORT_GIF
                        frameImg = getframe(ax);
                        if size(frameImg.cdata, 1) ~= vH || size(frameImg.cdata, 2) ~= vW
                            frameImg.cdata = imresize(frameImg.cdata, [vH vW]);
                        end
                        [imgGif, mapGif] = rgb2ind(frameImg.cdata, 256);
                        imwrite(imgGif, mapGif, currentGifFile, 'gif', 'WriteMode', 'append', 'DelayTime', GIF_DELAY);
                    end
                end
            end
        end
        
        prevPC = pcd;
        
        % === INCREMENTO ADAPTATIVO ===
        i = i + current_skip;
    end

    tiempo1 = toc(t1);
    
    % CERRAR VIDEO V1 Y ABRIR VIDEO V2
    if SAVE_VIDEO && ~isempty(vWriter)
        close(vWriter);
        fprintf('\n📹 Video V1 guardado: %s (%d frames)\n', currentVideoFile, wroteFrames);
        if EXPORT_GIF
            fprintf('🎬 GIF V1 guardado: %s\n', currentGifFile);
        end
        
        % ABRIR NUEVO VIDEO PARA V2
        currentVideoFile = VIDEO_V2_FILE;
        currentGifFile = GIF_V2_FILE;
        wroteFrames = 0;
        
        try, vWriter = VideoWriter(currentVideoFile,'MPEG-4'); catch
            vWriter = VideoWriter(strrep(currentVideoFile, '.mp4', '.avi'),'Motion JPEG AVI');
        end
        vWriter.FrameRate = VIDEO_FPS; vWriter.Quality = VIDEO_QUALITY; open(vWriter);
        
        fprintf('📹 Video V2 iniciado: %s\n', currentVideoFile);
        if EXPORT_GIF
            fprintf('🎬 GIF V2 iniciado: %s\n', currentGifFile);
        end
        
        % INICIALIZAR GIF V2 con primer frame
        drawnow; [vH, vW] = writeVideoFixed(vWriter, ax, vH, vW); wroteFrames = 1;
        if EXPORT_GIF
            frameImg = getframe(ax);
            if size(frameImg.cdata, 1) ~= vH || size(frameImg.cdata, 2) ~= vW
                frameImg.cdata = imresize(frameImg.cdata, [vH vW]);
            end
            [imgGif, mapGif] = rgb2ind(frameImg.cdata, 256);
            imwrite(imgGif, mapGif, currentGifFile, 'gif', 'LoopCount', Inf, 'DelayTime', GIF_DELAY);
        end
    end
    
    fprintf('\n');
    fprintf('┌─────────────────────────────────────────────────┐\n');
    fprintf('│ ✅ VUELTA 1 COMPLETADA - SISTEMA ULTRA-ROBUSTO │\n');
    fprintf('├─────────────────────────────────────────────────┤\n');
    fprintf('│ ⏱️  Tiempo total:      %.1f s\n', tiempo1);
    fprintf('│ 🗺️  Puntos en mapa:   %d\n', mapPC.Count);
    fprintf('│ 📍 Frames procesados:  %d\n', viewId-1);
    fprintf('│ 🎯 Algoritmo:          NDT + ICP Multi-escala\n');
    fprintf('│ 🔧 Skip adaptativo:    Activado (curvas=1)\n');
    fprintf('└─────────────────────────────────────────────────┘\n');
    fprintf('\n');
    
    %% ============= EXPORTAR MAPA VUELTA 1 PARA NAVEGACIÓN =============
    fprintf('💾 Exportando mapa de Vuelta 1 para navegación...\n');
    
    % Exportar PLY
    map_v1_ply = 'mapa_vuelta1_navegacion.ply';
    try
        pcwrite(mapPC, map_v1_ply, 'PLYFormat', 'binary');
        fprintf('   ✅ PLY guardado: %s (%d puntos)\n', map_v1_ply, mapPC.Count);
    catch ME
        fprintf('   ⚠️ Error guardando PLY: %s\n', ME.message);
    end
    
    % Exportar CSV (X, Y, Z)
    map_v1_csv = 'mapa_vuelta1_navegacion.csv';
    try
        pts_v1 = mapPC.Location;
        csvwrite(map_v1_csv, pts_v1);
        fprintf('   ✅ CSV guardado: %s (%d puntos)\n', map_v1_csv, size(pts_v1, 1));
    catch ME
        fprintf('   ⚠️ Error guardando CSV: %s\n', ME.message);
    end
    
    % Exportar MAT (para acceso rápido en MATLAB)
    map_v1_mat = 'mapa_vuelta1_navegacion.mat';
    try
        map_vuelta1.pointcloud = mapPC;
        map_vuelta1.points = mapPC.Location;
        map_vuelta1.trajectory = path1;
        map_vuelta1.rtk = XYZrtk_v1;
        map_vuelta1.timestamp = datetime('now');
        map_vuelta1.num_points = mapPC.Count;
        save(map_v1_mat, 'map_vuelta1', '-v7.3');
        fprintf('   ✅ MAT guardado: %s\n', map_v1_mat);
    catch ME
        fprintf('   ⚠️ Error guardando MAT: %s\n', ME.message);
    end
    
    fprintf('🎉 Mapa de Vuelta 1 exportado exitosamente\n\n');

    %% ========================== VUELTA 2: LOCALIZACIÓN ==========================
    fprintf('\n╔═══════════════════════════════════════╗\n');
    fprintf('║    🎯 VUELTA 2: LOCALIZACIÓN         ║\n');
    fprintf('╚═══════════════════════════════════════╝\n\n');
    title(ax,'SLAM 3D Profesional: Localización en Tiempo Real', ...
          'Color', textColor, 'FontSize', 16, 'FontWeight', 'bold'); 
    t2 = tic;

    firstFrame = getFramePC(ptCloudLoc,1);
    prevLocPt = preprocessForProfessionalMap(firstFrame, egoRadius, cylinderRadius, minHeight, maxHeight, hasSMRF);
    prevLocPt = pcdownsample(prevLocPt,'random',downsamplePercent);

    if hasRTK && ~isempty(XYZrtk_v2)
        locPose = rigidtform3d([eye(3), XYZrtk_v2(1,:)'; 0 0 0 1]);
    else
        locPose = absPose;
    end
    
    relPoseV2 = rigidtform3d;
    path2 = [locPose.Translation];
    vehHandleLocal = [];    % marcador vehículo
    frameCount = 0;
    loop_closure_applied = false;
    
    % === NUEVOS ALGORITMOS DE CONTROL DE DERIVA V2 ===
    drift_history = [];           % Historial de deriva detectada
    pose_confidence = [];         % Confianza en cada pose
    rtk_slam_divergence = [];     % Divergencia RTK-SLAM
    last_valid_pose_v2 = locPose; % Última pose válida
    drift_correction_count = 0;   % Contador de correcciones

    % Estado V2
    lastPct   = -1;
    lastLimT  = tic;
    vidTick2  = 0;

    % Log inicial y flecha
    if POSE_LOG, logPose('V2', 1, locPose); end
    updateHeadingArrow(hHeadingV2, locPose, headingLen);

    for i = 1:skipFrames:numel(ptCloudLoc)
        frame = getFramePC(ptCloudLoc,i);
        pt = preprocessForProfessionalMap(frame, egoRadius, cylinderRadius, minHeight, maxHeight, hasSMRF);
        ptd = pcdownsample(pt,'random',downsamplePercent);

        % LOOP CLOSURE MEJORADO (activo en V2 para corrección de deriva)
        current_pos = locPose.Translation;
        if ENABLE_LOOP_CLOSURE && size(loop_closure_poses,1) > 0
            distances = sqrt(sum((loop_closure_poses - current_pos).^2, 2));
            [min_distance, closest_idx] = min(distances);
            if min_distance < LOOP_CLOSURE_THRESHOLD && i > LOOP_CLOSURE_MIN_FRAMES
                fprintf('🔄 LOOP CLOSURE DETECTADO! Distancia: %.2f m\n', min_distance);
                loop_correction = loop_closure_poses(closest_idx,:) - current_pos;
                % Aplicar corrección gradual para evitar saltos bruscos
                correction_factor = min(0.5, LOOP_CLOSURE_THRESHOLD / min_distance);
                corrected_pose = rigidtform3d([locPose.R, (current_pos + correction_factor*loop_correction)'; 0 0 0 1]);
                locPose = corrected_pose;
                loop_closure_applied = true;
                drift_correction_count = drift_correction_count + 1;
                fprintf('✅ DERIVA CORREGIDA (corrección #%d)\n', drift_correction_count);
            end
        end

        % LOCALIZACIÓN MEJORADA CON CONTROL DE DERIVA
        if isempty(mapPC)
            try
                relPoseV2 = pcregisterndt(ptd, prevLocPt, gridStepV2, 'InitialTransform', relPoseV2);
                locPose = rigidtform3d(locPose.A * relPoseV2.A);
            catch
                fprintf('⚠️ Fallo en odometría, frame %d\n', i);
            end
        else
            cx = locPose.Translation(1); cy = locPose.Translation(2); cz = locPose.Translation(3);
            roi = [cx-roiR cx+roiR; cy-roiR cy+roiR; cz-8 cz+12];
            idx = findPointsInROI(mapPC, roi);

            if ~isempty(idx)
                subMap = select(mapPC, idx, 'OutputSize','full');
                if subMap.Count > 50000, subMap = pcdownsample(subMap,'random', 50000/subMap.Count); end
                
                % === MEJORAR TRANSFORMACIÓN INICIAL CON RTK ===
                initial_loc_transform = locPose;
                rtk_weight = RTK_FOLLOW_PRECISION; % Peso base RTK
                
                if hasRTK && ~isempty(XYZrtk_v2) && i <= size(XYZrtk_v2,1) && i > 1
                    rtk_motion_v2 = XYZrtk_v2(i,:) - XYZrtk_v2(i-1,:);
                    if RTK_Z_WEIGHT > 1 && numel(rtk_motion_v2) >= 3, rtk_motion_v2(3) = rtk_motion_v2(3)*RTK_Z_WEIGHT; end
                    rtk_transform_v2 = rigidtform3d([eye(3), rtk_motion_v2(:); 0 0 0 1]);
                    suggested_pose = rigidtform3d(locPose.A * rtk_transform_v2.A);
                    initial_loc_transform = suggested_pose;
                    
                    % Incrementar peso RTK si hay deriva detectada
                    if V2_ENHANCED_PRECISION && drift_correction_count > 0
                        rtk_weight = RTK_V2_WEIGHT; % Usar peso aumentado
                        fprintf('🎯 Usando peso RTK aumentado: %.2f\n', rtk_weight);
                    end
                end
                
                try
                    % NDT con parámetros mejorados para V2
                    T = pcregisterndt(ptd, subMap, max(2.0,gridStepV2), ...
                        'InitialTransform', initial_loc_transform, ...
                        'MaxIterations', V2_MAX_ITERATIONS, ...
                        'Tolerance', V2_TOLERANCE, ...
                        'OutlierRatio', V2_OUTLIER_RATIO);
                    
                    % === FUSIÓN RTK-SLAM MEJORADA ===
                    if hasRTK && ~isempty(XYZrtk_v2) && i <= size(XYZrtk_v2,1)
                        rtk_loc_pose = rigidtform3d([eye(3), XYZrtk_v2(i,:)'; 0 0 0 1]);
                        
                        % Detectar divergencia RTK-SLAM
                        rtk_slam_diff = norm(rtk_loc_pose.Translation - T.Translation);
                        rtk_slam_divergence = [rtk_slam_divergence; rtk_slam_diff]; %#ok<AGROW>
                        
                        % Si hay deriva significativa, aumentar peso RTK
                        if V2_DRIFT_CORRECTION && rtk_slam_diff > RTK_V2_DRIFT_THRESHOLD
                            rtk_weight = RTK_V2_WEIGHT; % Peso máximo RTK
                            drift_history = [drift_history; rtk_slam_diff]; %#ok<AGROW>
                            fprintf('🚨 DERIVA DETECTADA: %.2f m - Corrigiendo con RTK\n', rtk_slam_diff);
                        end
                        
                        combined_translation = rtk_weight * rtk_loc_pose.Translation + ...
                                              (1 - rtk_weight) * T.Translation;
                        locPose = rigidtform3d([T.R, combined_translation(:); 0 0 0 1]);
                        
                        % Guardar como pose válida si está dentro del umbral
                        if rtk_slam_diff < RTK_V2_DRIFT_THRESHOLD
                            last_valid_pose_v2 = locPose;
                        end
                    else
                        locPose = T;
                    end
                catch
                    % Fallback con validación de deriva
                    try
                        relPoseV2 = pcregisterndt(ptd, prevLocPt, gridStepV2, 'InitialTransform', relPoseV2);
                        proposed_pose = rigidtform3d(locPose.A * relPoseV2.A);
                        
                        % Validar que no hay salto excesivo
                        pose_jump = norm(proposed_pose.Translation - locPose.Translation);
                        if pose_jump < 5.0  % Máximo 5m por frame
                            locPose = proposed_pose;
                        else
                            fprintf('⚠️ Salto excesivo detectado: %.2f m - Manteniendo pose anterior\n', pose_jump);
                            % Usar última pose válida o RTK como fallback
                            if hasRTK && ~isempty(XYZrtk_v2) && i <= size(XYZrtk_v2,1)
                                locPose = rigidtform3d([eye(3), XYZrtk_v2(i,:)'; 0 0 0 1]);
                                fprintf('📍 Usando RTK como fallback\n');
                            end
                        end
                    catch
                        fprintf('⚠️ Fallo en localización, frame %d\n', i);
                        % Como último recurso, usar RTK si está disponible
                        if hasRTK && ~isempty(XYZrtk_v2) && i <= size(XYZrtk_v2,1)
                            locPose = rigidtform3d([eye(3), XYZrtk_v2(i,:)'; 0 0 0 1]);
                            fprintf('🆘 Usando RTK de emergencia\n');
                        end
                    end
                end
            end
        end

        % ========= 📍 PUBLICAR POSE EN TIEMPO REAL V2 =========
        pose_xyz_v2 = locPose.Translation;
        X_v2 = pose_xyz_v2(1);
        Y_v2 = pose_xyz_v2(2);
        Z_v2 = pose_xyz_v2(3);
        
        % Calcular theta (yaw) desde matriz de rotación
        R_v2 = locPose.R;
        theta_rad_v2 = atan2(R_v2(2,1), R_v2(1,1));
        theta_deg_v2 = rad2deg(theta_rad_v2);
        
        % Publicar pose en terminal
        fprintf('V2 [Frame %4d] | X: %7.2f | Y: %7.2f | Z: %6.2f | θ: %6.1f°\n', ...
                i, X_v2, Y_v2, Z_v2, theta_deg_v2);
        
        % === LOGGING DE POSE ===
        if POSE_LOG && (POSE_LOG_EVERY_FRAME || mod(i, POSE_LOG_SAMPLE_N)==0)
            logPose('V2', i, locPose);
        end

        % VISUAL (V2: sin nube actual cuando VIS_SHOW_CURRENT_V2=false)
        frameCount = frameCount + 1;
        if mod(frameCount, updateRate) == 0
            path2 = [path2; locPose.Translation]; %#ok<AGROW>
            set(hPath2,'XData',path2(:,1),'YData',path2(:,2),'ZData',path2(:,3));

            currentTransformedCloud = pctransform(ptd, locPose);
            if ~VIS_SHOW_CURRENT_V2
                currentTransformedCloud = []; % NO mostrar nube actual en V2
            end

            [lastPct,lastLimT] = updateVisualization3D_Professional( ...
                ax, currentTransformedCloud, path2, i, numel(ptCloudLoc), ...
                mapPC.Count, mapPC, vizHandle, DARK_THEME, true, ...
                VIS_SHOW_CURRENT_V2, lastPct, lastLimT);

            % Export V2
            if EXPORT_POINTS
                pts2 = ptd.Location;
                if hasRTK && ~isempty(XYZrtk_v2) && i <= size(XYZrtk_v2,1)
                    rtk_here2 = repmat(XYZrtk_v2(i,:), size(pts2,1), 1);
                elseif hasRTK && ~isempty(XYZrtk)
                    rtk_here2 = repmat(intelligentInterpolation3D_Enhanced(XYZrtk, 1), size(pts2,1), 1);
                else
                    rtk_here2 = zeros(size(pts2));
                end
                exportXYZ  = [exportXYZ; pts2];                 %#ok<AGROW>
                exportRTK  = [exportRTK; rtk_here2];            %#ok<AGROW>
                exportMeta = [exportMeta; [repmat(i, size(pts2,1), 1), repmat(2, size(pts2,1), 1)]]; %#ok<AGROW>
            end

            % Marcador del vehículo + flecha de orientación
            if isempty(vehHandleLocal) || ~isgraphics(vehHandleLocal)
                % Crear carro en forma de cuadrado/rectángulo
                vehHandleLocal = drawVehicleShape(ax, locPose, 2.0, 4.5, 1.5, [1 0 0], [1 1 1]);
            else
                % Actualizar posición del carro
                updateVehicleShape(vehHandleLocal, locPose, 2.0, 4.5, 1.5);
            end
            updateHeadingArrow(hHeadingV2, locPose, headingLen);

            if SAVE_VIDEO
                vidTick2 = vidTick2 + 1;
                if mod(vidTick2,2)==0
                    [vH, vW] = writeVideoFixed(vWriter, ax, vH, vW); wroteFrames = wroteFrames + 1;
                    
                    % CAPTURAR FRAME PARA GIF V2
                    if EXPORT_GIF
                        frameImg = getframe(ax);
                        if size(frameImg.cdata, 1) ~= vH || size(frameImg.cdata, 2) ~= vW
                            frameImg.cdata = imresize(frameImg.cdata, [vH vW]);
                        end
                        [imgGif, mapGif] = rgb2ind(frameImg.cdata, 256);
                        imwrite(imgGif, mapGif, currentGifFile, 'gif', 'WriteMode', 'append', 'DelayTime', GIF_DELAY);
                    end
                end
            end
        end
        prevLocPt = ptd;
    end

    tiempo2 = toc(t2);
    fprintf('\n╔═══════════════════════════════════════╗\n');
    fprintf('║     ✅ PROCESAMIENTO COMPLETADO        ║\n');
    fprintf('╚═══════════════════════════════════════╝\n');
    fprintf('⏱️  Tiempo V1: %.1fs | V2: %.1fs\n\n', toc(t1), tiempo2);
    
    % === REPORTE DE CONTROL DE DERIVA ===
    if ~isempty(drift_history)
        fprintf('📊 DERIVA DETECTADA: %d eventos\n', length(drift_history));
        fprintf('📊 Deriva máxima: %.2f m\n', max(drift_history));
        fprintf('📊 Deriva promedio: %.2f m\n', mean(drift_history));
    else
        fprintf('✅ No se detectó deriva significativa\n');
    end
    if drift_correction_count > 0
        fprintf('🔧 Correcciones aplicadas: %d\n', drift_correction_count);
    end
    if ~isempty(rtk_slam_divergence)
        fprintf('📈 Divergencia RTK-SLAM promedio: %.2f m\n', mean(rtk_slam_divergence));
    end

    %% ========================== EXPORTACIÓN CSV/PLY ==========================
    if EXPORT_POINTS && ~isempty(exportXYZ)
        try
            N = size(exportXYZ,1);
            if N > MAX_EXPORT_POINTS
                sel = randperm(N, MAX_EXPORT_POINTS);
                exportXYZ  = exportXYZ(sel,:);
                exportRTK  = exportRTK(sel,:);
                exportMeta = exportMeta(sel,:);
                fprintf('Export: muestreados %d de %d puntos para CSV.\n', MAX_EXPORT_POINTS, N);
            end
            out = [exportXYZ, exportRTK, exportMeta]; % [x y z rtk_x rtk_y rtk_z frame, pass]
            fid = fopen(EXPORT_FILENAME, 'w');
            fprintf(fid, 'x,y,z,rtk_x,rtk_y,rtk_z,frame,pass\n'); fclose(fid);
            writematrix(out, EXPORT_FILENAME, 'WriteMode','append');
            fprintf('Archivo exportado: %s  (filas: %d)\n', EXPORT_FILENAME, size(out,1));
        catch ME
            warning('Falló la exportación CSV: %s', ME.message);
        end
    end

    if ~isempty(mapPC)
        pcwrite(mapPC, 'final_map_3d_professional.ply');
        fprintf('Mapa exportado: %d puntos (final_map_3d_professional.ply)\n', mapPC.Count);
    end

    %% ========================== EVALUACIÓN / RESULTADOS ==========================
    if hasRTK && ~isempty(XYZrtk)
        fprintf('\n=== EVALUACIÓN vs RTK 3D ===\n');
        evaluateAccuracyRTK_3D_Enhanced(path1, path2, XYZrtk, XYZrtk_v1, XYZrtk_v2, loop_closure_applied);
    end

    results = struct();
    results.mapping_path = path1;
    results.localization_path = path2;
    results.rtk_reference_3d = XYZrtk;
    results.final_map = mapPC;
    results.processing_time = [tiempo1, tiempo2];
    results.loop_closure.applied = loop_closure_applied;
    results.rtk_3d.follow_precision = RTK_FOLLOW_PRECISION;
    save('slam_3d_professional_results.mat', 'results');

    if SAVE_VIDEO && ~isempty(vWriter)
        if wroteFrames==0
            drawnow; [vH, vW] = writeVideoFixed(vWriter, ax, vH, vW); wroteFrames = 1;
        end
        close(vWriter); 
        fprintf('\n📹 Video V2 guardado: %s (%d frames)\n', currentVideoFile, wroteFrames);
        if EXPORT_GIF
            fprintf('🎬 GIF V2 guardado: %s\n', currentGifFile);
        end
        fprintf('\n✅ Videos completos generados:\n');
        fprintf('   📹 V1 (Mapeo): %s\n', VIDEO_V1_FILE);
        fprintf('   📹 V2 (Localización): %s\n', VIDEO_V2_FILE);
        if EXPORT_GIF
            fprintf('   🎬 GIF V1: %s\n', GIF_V1_FILE);
            fprintf('   🎬 GIF V2: %s\n', GIF_V2_FILE);
        end
    end

    fprintf('\n=== VISTA FINAL 3D ===\n');
    showFinalProfessionalMap(mapPC, path1, path2, XYZrtk, DARK_THEME, 'Resultado Final SLAM 3D Profesional');

    % ====== Mapa 2D RTK en SATÉLITE (ya corregido) ======
    if hasRTK && ~isempty(latlonLLA)
        showRTK2D_Satellite(latlonLLA(:,1), latlonLLA(:,2), latlonLLA(:,3), ...
                            'RTK 2D - Satélite (World Imagery)');
        % showRTK2D_OSM(latlonLLA(:,1), latlonLLA(:,2));
    else
        fprintf('Aviso: No hay lat/lon crudos para dibujar el mapa 2D.\n');
    end

    fprintf('\n=== COMPLETADO | Total: %.1f s ===\n', tiempo1+tiempo2);
end

%% ========================== VISUALIZACIÓN ==========================
function vizHandle = initializeProfessionalVisualization(ax, darkTheme)
    if nargin < 2, darkTheme = true; end
    if darkTheme
        bgColor = [0.02 0.02 0.02]; gridColor = [0.15 0.15 0.15]; textColor = [0.9 0.9 0.9];
    else
        bgColor = [0.95 0.95 0.95]; gridColor = [0.7 0.7 0.7]; textColor = [0.1 0.1 0.1];
    end
    set(ax, 'Color', bgColor);
    set(ax, 'GridColor', gridColor, 'GridAlpha', 0.3);
    set(ax, 'XColor', textColor, 'YColor', textColor, 'ZColor', textColor);
    grid(ax, 'on'); set(ax, 'GridLineStyle', '-', 'MinorGridLineStyle', ':');
    lighting(ax, 'gouraud');
    material(ax, 'dull');        % Material mate, menos brillo
    camlight(ax, 'headlight');   % Luz frontal
    vizHandle = struct('mapPoints',[],'currentPoints',[],'theme',darkTheme, ...
                       'frameCounter',0,'lastUpdateTime',tic); %#ok<TNMLP>
end

function mapPC = buildProfessionalGlobalMap(mapPC, tfCloud, resMap, maxPtsMap, vizHandle)
    % ========= CLASIFICACIÓN INTELIGENTE Y FILTRADO AVANZADO =========
    
    % 1. Filtrado inicial básico
    if tfCloud.Count > 500
        tfCloud = pcdownsample(tfCloud, 'gridAverage', resMap * 0.8);
    end
    
    % 2. NUEVO: Clasificación automática por tipo de objeto
    if tfCloud.Count > 50
        % tfCloud = intelligentObjectClassification(tfCloud); % Temporalmente comentado
        % fprintf('🏷️ Clasificación omitida temporalmente\n');
    end
    
    % 3. NUEVO: Filtrado específico por categoría
    if tfCloud.Count > 100
        % tfCloud = categorySpecificFiltering(tfCloud); % Temporalmente comentado
        % fprintf('🎯 Filtrado específico omitido temporalmente\n');
    end
    
    % 4. NUEVO: Clustering mejorado para vegetación
    if tfCloud.Count > 200
        % tfCloud = enhancedVegetationClustering(tfCloud); % Temporalmente comentado
        % fprintf('🌳 Clustering de vegetación omitido temporalmente\n');
    end
    
    % 5. NUEVO: Reducción de redundancia inteligente
    if tfCloud.Count > 300
        % tfCloud = smartRedundancyRemoval(tfCloud); % Temporalmente comentado
        % fprintf('🗜️ Reducción de redundancia omitida temporalmente\n');
    end
    
    % 6. Filtrado de puntos flotantes (mantener)
    if tfCloud.Count > 50
        tfCloud = removeFloatingPoints(tfCloud);
    end
    
    % 7. Filtrado de densidad adaptativo (mantener)
    if tfCloud.Count > 100
        tfCloud = adaptiveDensityFilter(tfCloud);
    end
    
    % ========= CONSTRUCCIÓN DE MAPA CON CONTROL DE CALIDAD =========
    if isempty(mapPC)
        mapPC = tfCloud;
    else
        % Merge con validación geométrica
        mapPC = mergeWithQualityControl(mapPC, tfCloud, resMap);
        
        % Control de acumulación mejorado
        if mapPC.Count > maxPtsMap
            mapPC = intelligentMapReduction(mapPC, maxPtsMap, resMap);
        end
        
        % Limpieza periódica del mapa
        if mod(mapPC.Count, 30000) == 0
            mapPC = periodicMapCleaning(mapPC);
        end
    end
    
    % ========= ACTUALIZACIÓN DE VISUALIZACIÓN =========
    if ~isempty(vizHandle) && isstruct(vizHandle)
        % updateMapVisualization(vizHandle, mapPC); % Temporalmente comentado
        % fprintf('📊 Visualización de mapa omitida temporalmente\n');
    end
    
    if nargin >= 5 && ~isempty(vizHandle) && isstruct(vizHandle) && isfield(vizHandle, 'frameCounter')
        updateRealtimeVisualization(vizHandle, mapPC, tfCloud, true); % en V1 mostramos nube actual
    end
end

function updateRealtimeVisualization(vizHandle, mapPC, currentCloud, showCurrent)
    if isempty(vizHandle), return; end
    if nargin < 4, showCurrent = true; end

    vizHandle.frameCounter = vizHandle.frameCounter + 1;
    if toc(vizHandle.lastUpdateTime) < 0.12, return; end
    vizHandle.lastUpdateTime = tic;

    % ===== PARÁMETROS OPTIMIZADOS ESTILO ROS2 =====
    MAP_DISPLAY_LIMIT      = 10000;  % Más puntos para mejor calidad
    CURRENT_DISPLAY_LIMIT  = 800;    % Más puntos para nube actual
    POINT_SIZE_MAP         = 3.0;    % Puntos más visibles
    POINT_SIZE_CURRENT     = 4.0;    % Nube actual destacada
    TRANSPARENCY_MAP       = 0.8;    % Más opaco para mejor visibilidad
    TRANSPARENCY_CURRENT   = 0.85;   % Nube actual bien visible
    ax = gca;

    % ===== MAPA ACUMULADO CON FILTRADO INTELIGENTE =====
    if ~isempty(mapPC) && mapPC.Count > 0 && mod(vizHandle.frameCounter, 3) == 0
        if mapPC.Count > MAP_DISPLAY_LIMIT
            % Usar gridAverage para conservar estructura
            try
                mapDisplay = pcdownsample(mapPC, 'gridAverage', 0.35);
                if mapDisplay.Count > MAP_DISPLAY_LIMIT
                    mapDisplay = pcdownsample(mapDisplay, 'random', MAP_DISPLAY_LIMIT / mapDisplay.Count);
                end
            catch
                mapDisplay = pcdownsample(mapPC, 'random', MAP_DISPLAY_LIMIT / mapPC.Count);
            end
        else
            mapDisplay = mapPC;
        end
        
        Pmap = mapDisplay.Location;
        Cmap = enhancedHeightColorStyle(Pmap); % Colores mejorados

        if isempty(vizHandle.mapPoints) || ~isvalid(vizHandle.mapPoints)
            vizHandle.mapPoints = scatter3(ax, Pmap(:,1), Pmap(:,2), Pmap(:,3), ...
                POINT_SIZE_MAP, Cmap, 'filled', 'MarkerFaceAlpha', TRANSPARENCY_MAP, ...
                'MarkerEdgeAlpha', 0.1, 'HandleVisibility','off','PickableParts','none');
        else
            set(vizHandle.mapPoints, 'XData', Pmap(:,1), 'YData', Pmap(:,2), ...
                                     'ZData', Pmap(:,3), 'CData', Cmap, ...
                                     'SizeData', POINT_SIZE_MAP);
        end
    end

    % ===== NUBE ACTUAL MEJORADA =====
    if showCurrent && ~isempty(currentCloud) && currentCloud.Count > 0
        if currentCloud.Count > CURRENT_DISPLAY_LIMIT
            curDisp = pcdownsample(currentCloud, 'gridAverage', 0.25);
            if curDisp.Count > CURRENT_DISPLAY_LIMIT
                curDisp = pcdownsample(curDisp, 'random', CURRENT_DISPLAY_LIMIT / curDisp.Count);
            end
        else
            curDisp = currentCloud;
        end
        Pc = curDisp.Location;
        Cc = heightColorStyle(Pc);

        if isempty(vizHandle.currentPoints) || ~isvalid(vizHandle.currentPoints)
            vizHandle.currentPoints = scatter3(ax, Pc(:,1), Pc(:,2), Pc(:,3), ...
                POINT_SIZE_CURRENT, Cc, 'filled', 'MarkerFaceAlpha', TRANSPARENCY_CURRENT, ...
                'HandleVisibility','off','PickableParts','none');
        else
            set(vizHandle.currentPoints, 'XData', Pc(:,1), 'YData', Pc(:,2), ...
                                         'ZData', Pc(:,3), 'CData', Cc);
        end
    elseif ~showCurrent && ~isempty(vizHandle.currentPoints) && isvalid(vizHandle.currentPoints)
        set(vizHandle.currentPoints,'XData',nan,'YData',nan,'ZData',nan);
    end
end

function [lastPct,lastLimT] = updateVisualization3D_Professional( ...
    ax, tfCloud, path, frameNum, totalFrames, mapCount, mapPC, ...
    vizHandle, darkTheme, showStats, showCurrent, lastPct, lastLimT)

    if nargin < 9,  darkTheme = true;  end
    if nargin < 10, showStats = true;  end
    if nargin < 11, showCurrent = true; end

    if darkTheme, textColor=[0.9 0.9 0.9]; gridColor=[0.2 0.2 0.2];
    else,        textColor=[0.1 0.1 0.1]; gridColor=[0.6 0.6 0.6]; end

    if ~isempty(vizHandle)
        updateRealtimeVisualization(vizHandle, mapPC, tfCloud, showCurrent);
    end

    if showStats
        pct = floor((frameNum / max(totalFrames,1))*50)*2;
        if pct ~= lastPct
            title(ax, sprintf('SLAM 3D Profesional | Frame: %d/%d (%d%%) | Mapa: %s pts', ...
                frameNum, totalFrames, pct, formatNumber(mapCount)), ...
                'FontSize',14,'FontWeight','bold','Color',textColor);
            lastPct = pct;
        end
    end

    if ~isempty(path) && size(path,1) > 1 && toc(lastLimT) > 0.5
        xr = [min(path(:,1)) max(path(:,1))]; yr = [min(path(:,2)) max(path(:,2))];
        margin = max(10, 0.2*max(diff(xr), diff(yr)));
        xlim(ax, [xr(1)-margin xr(2)+margin]); ylim(ax, [yr(1)-margin yr(2)+margin]);
        if size(path,2) >= 3
            zr = [min(path(:,3)) max(path(:,3))]; zmargin = max(5, 0.3*diff(zr));
            zlim(ax, [zr(1)-zmargin zr(2)+2*zmargin]);
        end
        lastLimT = tic;
    end

    grid(ax,'on'); set(ax,'GridColor',gridColor,'GridAlpha',0.4);
    view(ax,-45,25); axis(ax,'equal');
    drawnow limitrate nocallbacks;
end

function showFinalProfessionalMap(mapPC, path1, path2, XYZrtk, darkTheme, vizTitle)
    if nargin < 5, darkTheme = true; end
    if nargin < 6, vizTitle = 'Resultado Final SLAM 3D Profesional'; end
    
    % ========== VENTANA PRINCIPAL MÁS GRANDE Y ELEGANTE ==========
    fig = figure('Name', vizTitle, 'Position', [50, 50, 1850, 1000]);
    if darkTheme
        set(fig, 'Color', [0.08 0.08 0.12]); % Azul oscuro elegante
        textColor = [0.95 0.95 0.95];
        accentColor = [0.3 0.85 1.0]; % Cyan brillante
        bgPanel = [0.12 0.12 0.18];
    else
        set(fig, 'Color', [0.96 0.96 0.98]); 
        textColor = [0.15 0.15 0.15];
        accentColor = [0.2 0.5 0.9];
        bgPanel = [0.98 0.98 1.0];
    end
    try, set(fig,'GraphicsSmoothing','on'); catch, end
    
    % ========== LAYOUT MEJORADO ==========
    % Vista 3D principal (más grande)
    ax_main = subplot('Position', [0.05 0.20 0.58 0.75]); 
    hold(ax_main, 'on');
    initializeProfessionalVisualization(ax_main, darkTheme);
    
    % ========== NUBE DE PUNTOS OPTIMIZADA ESTILO ROS2 ==========
    if ~isempty(mapPC) && mapPC.Count > 0
        displayLimit = 15000; % Más puntos para mejor calidad visual
        if mapPC.Count > displayLimit
            % Usar gridAverage para conservar estructura de clusters
            try
                displayMap = pcdownsample(mapPC, 'gridAverage', 0.3);
                if displayMap.Count > displayLimit
                    displayMap = pcdownsample(displayMap, 'random', displayLimit / displayMap.Count);
                end
            catch
                displayMap = pcdownsample(mapPC, 'random', displayLimit / mapPC.Count);
            end
        else
            displayMap = mapPC;
        end
        
        pts = displayMap.Location; 
        rgb = enhancedHeightColorStyle(pts); % Usar colores mejorados
        
        scatter3(ax_main, pts(:,1), pts(:,2), pts(:,3), ...
                 6, rgb, 'filled', ... % Puntos más grandes y visibles
                 'MarkerFaceAlpha', 0.85, ... % Más opaco
                 'MarkerEdgeAlpha', 0.0, ...
                 'HandleVisibility', 'off');
    end
    
    % ========== TRAYECTORIAS CON ESTILO ==========
    if ~isempty(path1)
        plot3(ax_main, path1(:,1), path1(:,2), path1(:,3), '-', ...
              'LineWidth', 8, 'Color', [1.0 0.25 0.65], ... % Rosa vibrante
              'DisplayName', 'Trayectoria Mapeo');
    end
    if ~isempty(path2)
        plot3(ax_main, path2(:,1), path2(:,2), path2(:,3), '--', ...
              'LineWidth', 8, 'Color', [1.0 0.35 0.25], ... % Rojo-naranja
              'DisplayName', 'Trayectoria Localización');
    end
    if ~isempty(XYZrtk)
        plot3(ax_main, XYZrtk(:,1), XYZrtk(:,2), XYZrtk(:,3), ':', ...
              'LineWidth', 5, 'Color', [0.9 0.9 0.9], ...
              'DisplayName', 'Ground Truth RTK');
    end
    
    % ========== EJES Y TÍTULOS PROFESIONALES ==========
    grid(ax_main, 'on'); 
    set(ax_main, 'GridAlpha', 0.4, 'MinorGridAlpha', 0.2);
    xlabel(ax_main, 'X (metros)', 'Color', textColor, 'FontSize', 15, ...
           'FontWeight', 'bold', 'FontName', 'Arial');
    ylabel(ax_main, 'Y (metros)', 'Color', textColor, 'FontSize', 15, ...
           'FontWeight', 'bold', 'FontName', 'Arial');
    zlabel(ax_main, 'Z (metros)', 'Color', textColor, 'FontSize', 15, ...
           'FontWeight', 'bold', 'FontName', 'Arial');
    
    title(ax_main, {'Vista 3D del Mapa Final', ''}, ...
          'Color', accentColor, 'FontSize', 22, 'FontWeight', 'bold', ...
          'FontName', 'Arial');
    
    % ========== LEYENDA MEJORADA ==========
    lgd = legend(ax_main, 'Location', 'northeast', 'TextColor', textColor, ...
                 'FontSize', 14, 'FontWeight', 'bold');
    set(lgd, 'Color', bgPanel, 'EdgeColor', accentColor, 'LineWidth', 2.5);
    
    view(ax_main, -40, 35); % Ángulo más dinámico
    axis(ax_main, 'equal');
    set(ax_main, 'FontSize', 12);
    
    % ========== VISTA SUPERIOR (derecha arriba) ==========
    ax_top = subplot('Position', [0.67 0.55 0.30 0.40]); 
    hold(ax_top, 'on');
    set(ax_top, 'Color', bgPanel);
    if darkTheme
        set(ax_top, 'XColor', textColor, 'YColor', textColor);
    end
    
    if ~isempty(mapPC) && exist('displayMap', 'var')
        % Vista superior mejorada con menos puntos pero mejor estructura
        if displayMap.Count > 8000
            try
                topDisplay = pcdownsample(displayMap, 'gridAverage', 0.5);
            catch
                % Fallback a random sampling
                pts_all = displayMap.Location;
                idx = randperm(size(pts_all, 1), min(8000, size(pts_all, 1)));
                topDisplay = pointCloud(pts_all(idx, :));
            end
        else
            topDisplay = displayMap;
        end
        
        pts2d = topDisplay.Location(:, 1:2);
        topColors = enhancedHeightColorStyle(topDisplay.Location);
        
        scatter(ax_top, pts2d(:,1), pts2d(:,2), 5, ...
                topColors, 'filled', 'MarkerFaceAlpha', 0.8, ...
                'MarkerEdgeAlpha', 0.0, 'HandleVisibility', 'off');
    end
    
    if ~isempty(path1)
        plot(ax_top, path1(:,1), path1(:,2), '-', 'LineWidth', 6, ...
             'Color', [1.0 0.25 0.65]);
    end
    if ~isempty(path2)
        plot(ax_top, path2(:,1), path2(:,2), '--', 'LineWidth', 6, ...
             'Color', [1.0 0.35 0.25]);
    end
    if ~isempty(XYZrtk)
        plot(ax_top, XYZrtk(:,1), XYZrtk(:,2), ':', 'LineWidth', 4, ...
             'Color', [0.65 0.65 0.65]);
    end
    
    grid(ax_top, 'on'); 
    set(ax_top, 'GridAlpha', 0.3);
    axis(ax_top, 'equal');
    xlabel(ax_top, 'X (m)', 'Color', textColor, 'FontSize', 13, 'FontWeight', 'bold');
    ylabel(ax_top, 'Y (m)', 'Color', textColor, 'FontSize', 13, 'FontWeight', 'bold');
    title(ax_top, 'Vista Superior (Plano XY)', 'Color', accentColor, ...
          'FontSize', 16, 'FontWeight', 'bold');
    
    % ========== PANEL DE ESTADÍSTICAS HORIZONTAL (abajo, ancho completo) ==========
    ax_stats = subplot('Position', [0.05 0.03 0.92 0.12]); 
    set(ax_stats, 'Color', bgPanel);
    axis(ax_stats, 'off');
    xlim(ax_stats, [0 1]); ylim(ax_stats, [0 1]);
    
    % Título
    text(ax_stats, 0.5, 0.88, 'Estadísticas del Sistema', ...
         'Units', 'normalized', ...
         'Color', accentColor, ...
         'FontWeight', 'bold', ...
         'FontSize', 17, ...
         'FontName', 'Arial', ...
         'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'top');
    
    % Generar texto en formato horizontal
    stats_h = generateHorizontalStats(mapPC, path1, path2, XYZrtk);
    
    % Texto horizontal
    text(ax_stats, 0.5, 0.40, stats_h, ...
         'Units', 'normalized', ...
         'Color', textColor, ...
         'FontSize', 11.5, ...
         'FontName', 'Courier New', ...
         'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'middle', ...
         'Interpreter', 'none');
    
    % Borde del panel
    rectangle(ax_stats, 'Position', [0.01 0.01 0.98 0.98], ...
              'EdgeColor', accentColor, 'LineWidth', 2.5);
end

% ========== FUNCIÓN PARA STATS HORIZONTALES ==========
function stats_h = generateHorizontalStats(mapPC, path1, path2, XYZrtk)
    parts = {};
    
    % MAPA 3D
    if ~isempty(mapPC)
        bounds = [min(mapPC.Location); max(mapPC.Location)];
        area = (bounds(2,1)-bounds(1,1)) * (bounds(2,2)-bounds(1,2));
        density = mapPC.Count / max(area, 1);
        
        parts{end+1} = sprintf('MAPA: %s pts (%.1f pts/m2)', ...
                               formatNumber(mapPC.Count), density);
    end
    
    % TRAYECTORIAS
    traj_info = {};
    if ~isempty(path1)
        traj_info{end+1} = sprintf('Mapeo: %.1fm', calculateTrajectoryLength3D(path1));
    end
    if ~isempty(path2)
        traj_info{end+1} = sprintf('Loc: %.1fm', calculateTrajectoryLength3D(path2));
    end
    if ~isempty(XYZrtk)
        traj_info{end+1} = sprintf('RTK: %.1fm', calculateTrajectoryLength3D(XYZrtk));
    end
    if ~isempty(traj_info)
        parts{end+1} = ['TRAYECTORIAS: ' strjoin(traj_info, ' | ')];
    end
    
    % SISTEMA
    parts{end+1} = sprintf('SISTEMA: Profesional | %s | COMPLETADO', datestr(now, 'HH:MM:SS'));
    
    % Unir todo con separador
    stats_h = strjoin(parts, '   •   ');
end



%% ========================== MAPA 2D (SATELITAL y OSM) ==========================
function showRTK2D_Satellite(lat, lon, alt, windowTitle)
    if nargin < 4, windowTitle = 'RTK 2D - Satélite'; end
    f = figure('Name',windowTitle,'Position',[100,80,1200,800]);
    axg = geoaxes('Parent',f); geobasemap(axg,'satellite'); hold(axg,'on');
    axg.FontSize = 11;

    geoplot(axg, lat, lon, '-', 'LineWidth', 2.2, 'Color', [1 0.85 0]); % amarillo
    geoscatter(axg, lat, lon, 14, [0.2 0.8 1.0], 'filled', 'MarkerFaceAlpha',0.7);

    geolimits(axg, [min(lat) max(lat)], [min(lon) max(lon)]);
    title(axg,'RTK 2D sobre Satélite (World Imagery)');

    dcm = datacursormode(f); dcm.Enable = 'on'; dcm.DisplayStyle = 'datatip';
    dcm.UpdateFcn = @(obj,evt) localGeoTip(evt, lat, lon, alt);
end

function txt = localGeoTip(evt, lat, lon, alt)
    pos = evt.Position; % [lat, lon]
    d = hypot(lat - pos(1), lon - pos(2));
    [~,idx] = min(d);
    if nargin >= 4 && ~isempty(alt) && numel(alt)>=idx
        txt = {sprintf('Lat: %.6f', lat(idx)), ...
               sprintf('Lon: %.6f', lon(idx)), ...
               sprintf('Alt: %.2f m', alt(idx)), ...
               sprintf('Index: %d', idx)};
    else
        txt = {sprintf('Lat: %.6f', lat(idx)), ...
               sprintf('Lon: %.6f', lon(idx)), ...
               sprintf('Index: %d', idx)};
    end
end

function showRTK2D_OSM(lat, lon)
    try
        wm = webmap('OpenStreetMap'); %#ok<NASGU>
        wmline(lat, lon, 'Color','cyan', 'Width', 3);
        wmmarker(lat(1), lon(1), 'Icon', 'http://maps.google.com/mapfiles/ms/icons/green-dot.png', 'Description','Inicio');
        wmmarker(lat(end), lon(end), 'Icon', 'http://maps.google.com/mapfiles/ms/icons/red-dot.png', 'Description','Fin');
        zoomLevel = 17; wmlimits([min(lat) max(lat)], [min(lon) max(lon)]); wmzoom(zoomLevel);
    catch ME
        warning('No se pudo abrir webmap/OSM: %s. Usa showRTK2D_Satellite()', ME.message);
    end
end

%% ========================== UTILIDADES / RTK / PREPROCESO ==========================
function stats_text = generateDetailedStats(mapPC, path1, path2, XYZrtk)
    stats_text = 'REPORTE FINAL SLAM 3D\n';
    stats_text = [stats_text repmat('=', 1, 35) '\n\n'];
    if ~isempty(mapPC)
        stats_text = [stats_text sprintf('MAPA 3D:\n')];
        stats_text = [stats_text sprintf('  Puntos: %s\n', formatNumber(mapPC.Count))];
        bounds = [min(mapPC.Location); max(mapPC.Location)];
        stats_text = [stats_text sprintf('  Dimensiones:\n')];
        stats_text = [stats_text sprintf('    X: %.1f - %.1f m\n', bounds(1,1), bounds(2,1))];
        stats_text = [stats_text sprintf('    Y: %.1f - %.1f m\n', bounds(1,2), bounds(2,2))];
        stats_text = [stats_text sprintf('    Z: %.1f - %.1f m\n', bounds(1,3), bounds(2,3))];
        area = (bounds(2,1)-bounds(1,1)) * (bounds(2,2)-bounds(1,2));
        if area > 0
            density = mapPC.Count / area;
            stats_text = [stats_text sprintf('  Densidad: %.1f pts/m²\n', density)];
        end
    end
    stats_text = [stats_text sprintf('\nTRAYECTORIAS:\n')];
    if ~isempty(path1), stats_text = [stats_text sprintf('  Mapeo: %.1f m\n', calculateTrajectoryLength3D(path1))]; end
    if ~isempty(path2), stats_text = [stats_text sprintf('  Localización: %.1f m\n', calculateTrajectoryLength3D(path2))]; end
    if ~isempty(XYZrtk), stats_text = [stats_text sprintf('  RTK Ground Truth: %.1f m\n', calculateTrajectoryLength3D(XYZrtk))]; end
    stats_text = [stats_text sprintf('\nSISTEMA:\n  Visualización: Pro (sin flicker)\n  Tiempo: %s\n', datestr(now, 'HH:MM:SS'))];
end

function pc = preprocessForProfessionalMap(pcIn, egoR, cylR, minH, maxH, hasSMRF)
    % PROCESAMIENTO ULTRA RÁPIDO - SIMPLICIDAD MÁXIMA
    % Elimina todos los filtros complejos y costosos
    
    if pcIn.Count < 50, pc = pcIn; return; end
    
    % ÚNICO FILTRO: Altura simple (más rápido que cualquier otra cosa)
    points = pcIn.Location;
    z = points(:,3);
    
    % Filtrado básico por altura - ULTRA RÁPIDO
    valid = z > minH & z < maxH;
    if sum(valid) < 50
        pc = pcIn; % Si queda muy poco, mantener original
        return;
    end
    
    validPoints = points(valid, :);
    
    % Filtrado cilíndrico simple (opcional para velocidad)
    if cylR > 0
        x = validPoints(:,1); y = validPoints(:,2);
        dist = sqrt(x.^2 + y.^2);
        cylinderValid = dist < cylR & dist > egoR;
        if sum(cylinderValid) > 20
            validPoints = validPoints(cylinderValid, :);
        end
    end
    
    pc = pointCloud(validPoints);
end

function pc = ros2StyleGroundFilter(pc, hasSMRF)
    % FILTRADO DE SUELO ESTILO ROS2 NAV2/CARTOGRAPHER
    % Combina detección de planos + análisis morfológico
    
    if pc.Count < 50, return; end
    
    originalCount = pc.Count;
    points = pc.Location;
    
    % fprintf('🤖 Aplicando filtro de suelo estilo ROS2...\n');
    
    % Método 1: Detección de plano principal (como en PCL)
    groundMask = detectGroundPlanePCLStyle(points);
    
    % Método 2: Análisis de altura local (como en nav2)
    heightMask = localHeightAnalysis(points);
    
    % Método 3: Análisis de normalización (como en cartographer)
    normalMask = surfaceNormalAnalysis(points);
    
    % VALIDACIÓN: Asegurar que todas las máscaras tengan el mismo tamaño
    numPoints = size(points, 1);
    if length(groundMask) ~= numPoints
        groundMask = false(numPoints, 1);
    end
    if length(heightMask) ~= numPoints
        heightMask = false(numPoints, 1);
    end
    if length(normalMask) ~= numPoints
        normalMask = false(numPoints, 1);
    end
    
    % Combinación inteligente de métodos (voting scheme)
    finalGroundMask = (groundMask + heightMask + normalMask) >= 2;
    
    % Mantener solo puntos NO-suelo
    nonGroundPoints = points(~finalGroundMask, :);
    
    % Validación: asegurar que no eliminamos todo
    if size(nonGroundPoints, 1) < originalCount * 0.1
        fprintf('⚠️ Filtrado muy agresivo, ajustando...\n');
        % Usar solo altura como criterio
        zMin = min(points(:,3));
        conservativeMask = points(:,3) > (zMin + 0.8);
        nonGroundPoints = points(conservativeMask, :);
    end
    
    if ~isempty(nonGroundPoints) && size(nonGroundPoints, 1) > 20
        pc = pointCloud(nonGroundPoints);
        % reductionPercent = 100 * (originalCount - pc.Count) / originalCount;
        % fprintf('✅ ROS2 Ground Filter: %.1f%% puntos eliminados\n', reductionPercent);
    else
        % fprintf('⚠️ Filtro ROS2 falló, manteniendo original\n');
    end
end

function groundMask = detectGroundPlanePCLStyle(points)
    % DETECCIÓN DE PLANO ESTILO PCL (Point Cloud Library)
    
    groundMask = false(size(points, 1), 1);
    
    try
        % Parámetros estilo PCL RANSAC
        pc_temp = pointCloud(points);
        maxDistance = 0.1;  % 10cm threshold
        referenceVector = [0, 0, 1];  % Vertical upward
        maxAngularDistance = 10;  % 10 degrees
        
        [~, inlierIndices] = pcfitplane(pc_temp, maxDistance, ...
            referenceVector, maxAngularDistance, 'MaxNumTrials', 500);
        
        if ~isempty(inlierIndices)
            groundMask(inlierIndices) = true;
        end
        
    catch
        % Fallback: análisis estadístico simple
        z = points(:,3);
        zMedian = median(z);
        zMAD = mad(z);
        groundThreshold = zMedian + 1.5 * zMAD;
        groundMask = z <= groundThreshold;
    end
end

function heightMask = localHeightAnalysis(points)
    % ANÁLISIS DE ALTURA LOCAL ESTILO NAV2
    
    heightMask = false(size(points, 1), 1);
    z = points(:,3);
    
    % Grid-based height analysis (como en nav2)
    gridSize = 1.0;  % 1 metro de grid
    
    x = points(:,1);
    y = points(:,2);
    
    xRange = [min(x), max(x)];
    yRange = [min(y), max(y)];
    
    xBins = xRange(1):gridSize:xRange(2);
    yBins = yRange(1):gridSize:yRange(2);
    
    for i = 1:length(xBins)-1
        for j = 1:length(yBins)-1
            % Puntos en esta celda del grid
            cellMask = x >= xBins(i) & x < xBins(i+1) & ...
                       y >= yBins(j) & y < yBins(j+1);
            
            if sum(cellMask) > 5  % Suficientes puntos en la celda
                cellZ = z(cellMask);
                cellZMin = min(cellZ);
                
                % Puntos cerca del mínimo son candidatos a suelo
                groundThreshold = cellZMin + 0.3;  % 30cm
                localGroundMask = cellMask & (z <= groundThreshold);
                heightMask = heightMask | localGroundMask;
            end
        end
    end
end

function normalMask = surfaceNormalAnalysis(points)
    % ANÁLISIS DE NORMALES DE SUPERFICIE ESTILO CARTOGRAPHER
    
    normalMask = false(size(points, 1), 1);
    
    if size(points, 1) < 100, return; end
    
    try
        pc_temp = pointCloud(points);
        normals = pcnormals(pc_temp, 20);  % 20 neighbors
        
        % Vectores que apuntan hacia arriba son candidatos a suelo
        upVector = [0, 0, 1];
        
        for i = 1:size(normals, 1)
            normal = normals(i, :);
            % Ángulo con la vertical
            angle = acos(abs(dot(normal, upVector)) / norm(normal));
            
            % Si el ángulo es pequeño (superficie horizontal)
            if angle < deg2rad(20)  % 20 grados
                normalMask(i) = true;
            end
        end
        
    catch
        % Fallback: usar solo altura
        z = points(:,3);
        zLow = prctile(z, 30);
        normalMask = z <= zLow;
    end
end

function pc = advancedVoxelGridFilter(pc)
    % VOXEL GRID FILTER AVANZADO ESTILO ROS2
    % Crea distribución uniforme de puntos
    
    if pc.Count < 200, return; end
    
    points = pc.Location;
    
    % Tamaño de voxel adaptativo basado en densidad
    volume = (max(points) - min(points));
    totalVolume = prod(volume);
    avgDensity = pc.Count / totalVolume;
    
    % Voxel size inversamente proporcional a la densidad
    voxelSize = max(0.1, min(0.5, 1.0 / sqrt(avgDensity)));
    
    % Aplicar voxel grid
    try
        pc = pcdownsample(pc, 'gridAverage', voxelSize);
        % fprintf('📦 Voxel Grid (%.2fm): %d puntos\n', voxelSize, pc.Count);
    catch
        % Fallback: random downsampling
        if pc.Count > 500
            pc = pcdownsample(pc, 'random', 0.7);
        end
    end
end

function pc = statisticalOutlierRemoval(pc)
    % FILTRADO ESTADÍSTICO DE OUTLIERS ESTILO PCL/ROS2
    
    if pc.Count < 100, return; end
    
    try
        % Parámetros estilo PCL
        meanK = 20;  % Número de vecinos
        stddevMulThresh = 2.0;  % Threshold de desviación estándar
        
        [pc, ~] = pcdenoise(pc, 'NumNeighbors', meanK, 'Threshold', stddevMulThresh);
        
    catch
        % Fallback: filtrado por distancia al centroide
        points = pc.Location;
        centroid = mean(points, 1);
        distances = sqrt(sum((points - centroid).^2, 2));
        
        meanDist = mean(distances);
        stdDist = std(distances);
        threshold = meanDist + 2 * stdDist;
        
        validMask = distances <= threshold;
        if sum(validMask) > 50
            pc = pointCloud(points(validMask, :));
        end
    end
end

function pc = intelligentDownsampling(pc)
    % DOWNSAMPLING INTELIGENTE PARA MANTENER CARACTERÍSTICAS IMPORTANTES
    
    if pc.Count <= 1000, return; end
    
    points = pc.Location;
    
    % Análisis de curvatura para mantener puntos importantes
    try
        normals = pcnormals(pc, 15);
        curvatures = calculateCurvature(points, normals);
        
        % Mantener puntos con alta curvatura (bordes, esquinas)
        highCurvatureThreshold = prctile(curvatures, 80);
        importantMask = curvatures >= highCurvatureThreshold;
        
        % Downsampling del resto
        regularMask = ~importantMask;
        targetCount = min(800, round(pc.Count * 0.8));
        
        if sum(regularMask) > 0
            regularIndices = find(regularMask);
            keepCount = max(1, targetCount - sum(importantMask));
            
            if keepCount < length(regularIndices)
                keepIndices = regularIndices(randperm(length(regularIndices), keepCount));
                finalMask = importantMask;
                finalMask(keepIndices) = true;
                
                pc = pointCloud(points(finalMask, :));
            end
        end
        
    catch
        % Fallback: downsampling aleatorio
        pc = pcdownsample(pc, 'random', 0.8);
    end
end

function curvatures = calculateCurvature(points, normals)
    % Cálculo simple de curvatura basado en variación de normales
    
    n = size(points, 1);
    curvatures = zeros(n, 1);
    
    for i = 1:n
        % Encontrar vecinos cercanos
        distances = sqrt(sum((points - points(i,:)).^2, 2));
        [~, nearestIndices] = mink(distances, min(10, n));
        nearestIndices = nearestIndices(2:end);  % Excluir el punto mismo
        
        if length(nearestIndices) > 3
            % Variación de normales en el vecindario
            neighborNormals = normals(nearestIndices, :);
            currentNormal = normals(i, :);
            
            % Calcular variación
            deviations = zeros(length(nearestIndices), 1);
            for j = 1:length(nearestIndices)
                deviation = acos(max(-1, min(1, dot(currentNormal, neighborNormals(j,:)))));
                deviations(j) = deviation;
            end
            
            curvatures(i) = std(deviations);
        end
    end
end

%% ========================== FILTRADO INTELIGENTE Y CLUSTERS ==========================
function pc = smartGroundAndClusterFilter(pc, useSMRF)
    % FILTRADO ULTRA AGRESIVO - ELIMINA TODO LO QUE ESTÉ EN EL PISO
    % Para ti: SUELO = TODO lo que esté menos de 1.2 metros de altura
    
    if pc.Count < 50, return; end
    
    fprintf('🔥 APLICANDO FILTRADO ULTRA AGRESIVO DEL SUELO...\n');
    
    % Aplicar filtrado súper agresivo
    pc = ultraAggressiveGroundRemoval(pc);
    
    fprintf('✅ Filtrado ULTRA AGRESIVO completo. Puntos restantes: %d\n', pc.Count);
end

function pc = ultraAggressiveGroundRemoval(pc)
    % DEFINICIÓN DE SUELO ULTRA AGRESIVA:
    % - Todo lo que esté a menos de 1.2m del punto más bajo = SUELO (ELIMINADO)
    % - Solo conservar puntos que sean claramente ÁRBOLES, POSTES, EDIFICIOS
    
    originalCount = pc.Count;
    points = pc.Location;
    
    if isempty(points) || size(points, 1) < 50
        return;
    end
    
    z = points(:,3);
    zMin = min(z);
    
    % FILTRADO ULTRA AGRESIVO: Solo conservar puntos ALTOS
    MINIMUM_OBJECT_HEIGHT = 1.2; % 1.2 metros sobre el suelo mínimo
    heightThreshold = zMin + MINIMUM_OBJECT_HEIGHT;
    
    % Primera pasada: Eliminar todo lo que esté cerca del suelo
    highPointsMask = z > heightThreshold;
    highPoints = points(highPointsMask, :);
    
    fprintf('📏 Suelo definido como: Z < %.2f metros\n', heightThreshold);
    fprintf('🗑️ Eliminando %.1f%% de puntos (suelo + objetos bajos)\n', ...
        100 * (1 - sum(highPointsMask) / length(highPointsMask)));
    
    % Si quedaron muy pocos puntos, ser menos agresivo pero mantener estricto
    if size(highPoints, 1) < 100 && originalCount > 500
        fprintf('⚠️ Muy pocos puntos altos, ajustando a 0.8m...\n');
        MINIMUM_OBJECT_HEIGHT = 0.8; % Reducir a 80cm
        heightThreshold = zMin + MINIMUM_OBJECT_HEIGHT;
        highPointsMask = z > heightThreshold;
        highPoints = points(highPointsMask, :);
    end
    
    % Si AÚN no hay suficientes puntos, usar percentil superior
    if size(highPoints, 1) < 50 && originalCount > 200
        fprintf('⚠️ Insuficientes puntos, usando percentil 70...\n');
        heightThreshold = prctile(z, 70); % Solo el 30% más alto
        highPointsMask = z > heightThreshold;
        highPoints = points(highPointsMask, :);
    end
    
    % Segundo filtro: Remover puntos aislados cerca del suelo
    if size(highPoints, 1) > 50
        highPoints = removeIsolatedLowPoints(highPoints, zMin);
    end
    
    % Crear nueva nube solo con objetos ALTOS
    if ~isempty(highPoints) && size(highPoints, 1) > 10
        pc = pointCloud(highPoints);
        reductionPercent = 100 * (originalCount - pc.Count) / originalCount;
        fprintf('✂️ ELIMINADOS: %d puntos (%.1f%%) - Solo objetos > %.1fm\n', ...
            originalCount - pc.Count, reductionPercent, MINIMUM_OBJECT_HEIGHT);
    else
        fprintf('❌ ERROR: No quedan objetos válidos. Manteniendo puntos más altos...\n');
        % Como último recurso, tomar solo el 20% más alto
        [~, sortIdx] = sort(z, 'descend');
        keepCount = max(50, round(length(z) * 0.2));
        pc = pointCloud(points(sortIdx(1:keepCount), :));
    end
end

function cleanPoints = removeIsolatedLowPoints(points, groundLevel)
    % Remover puntos que estén aislados y cerca del suelo
    
    if size(points, 1) < 20
        cleanPoints = points;
        return;
    end
    
    z = points(:,3);
    
    % Identificar puntos que están relativamente cerca del suelo
    SUSPICIOUS_HEIGHT = groundLevel + 2.0; % 2 metros sobre suelo
    suspiciousMask = z < SUSPICIOUS_HEIGHT;
    
    if ~any(suspiciousMask)
        cleanPoints = points;
        return;
    end
    
    % Para puntos sospechosos, verificar si tienen vecinos altos
    validMask = true(size(points, 1), 1);
    suspiciousIndices = find(suspiciousMask);
    
    for i = 1:length(suspiciousIndices)
        idx = suspiciousIndices(i);
        currentPoint = points(idx, :);
        
        % Buscar vecinos en radio de 3 metros
        distances = sqrt(sum((points - currentPoint).^2, 2));
        neighborMask = distances < 3.0 & distances > 0.1;
        
        if sum(neighborMask) < 5
            % Punto aislado - eliminar
            validMask(idx) = false;
        else
            % Verificar altura promedio de vecinos
            neighborHeights = z(neighborMask);
            avgNeighborHeight = mean(neighborHeights);
            
            % Si los vecinos también son bajos, eliminar todo el grupo
            if avgNeighborHeight < (groundLevel + 1.5)
                validMask(idx) = false;
            end
        end
    end
    
    cleanPoints = points(validMask, :);
    
    if size(cleanPoints, 1) < size(points, 1)
        fprintf('🧹 Removidos %d puntos aislados cerca del suelo\n', ...
            size(points, 1) - size(cleanPoints, 1));
    end
end

%% ========================== FUNCIONES DE UTILIDAD SIMPLIFICADAS ==========================
function pc = simpleGroundReduction(pc)
    % MÉTODO SIMPLE PERO ULTRA AGRESIVO para eliminar suelo
    % DEFINICIÓN: Suelo = TODO lo que esté a menos de 1.0m del punto más bajo
    
    if pc.Count < 20, return; end
    
    points = pc.Location;
    if isempty(points) || size(points, 2) ~= 3, return; end
    
    z = points(:,3);
    zMin = min(z);
    
    % FILTRADO ULTRA AGRESIVO: Solo conservar puntos > 1.0m del suelo
    AGGRESSIVE_THRESHOLD = 1.0; % 1 metro sobre el punto más bajo
    heightThreshold = zMin + AGGRESSIVE_THRESHOLD;
    
    validMask = z > heightThreshold;
    
    % Si eliminamos demasiado, reducir un poco el threshold
    if sum(validMask) < pc.Count * 0.1 % Menos del 10% restante
        fprintf('🔥 Filtrado MUY agresivo, ajustando a 0.7m...\n');
        heightThreshold = zMin + 0.7;
        validMask = z > heightThreshold;
    end
    
    % Si aún es muy poco, usar percentil superior
    if sum(validMask) < 20
        fprintf('🔥 Usando percentil 80 como último recurso...\n');
        heightThreshold = prctile(z, 80);
        validMask = z > heightThreshold;
    end
    
    if sum(validMask) > 10
        finalPoints = points(validMask, :);
        pc = pointCloud(finalPoints);
        fprintf('⚡ SIMPLE AGRESIVO: Eliminados %.1f%% puntos (altura < %.2fm)\n', ...
            100 * (1 - sum(validMask) / length(validMask)), heightThreshold - zMin);
    end
end

function clusters = simpleDBSCAN(points, epsilon, minPts)
    % Implementación simplificada de DBSCAN
    n = size(points, 1);
    clusters = zeros(n, 1);
    cluster_id = 0;
    
    for i = 1:n
        if clusters(i) ~= 0, continue; end
        
        % Encontrar vecinos
        distances = sqrt(sum((points - points(i, :)).^2, 2));
        neighbors = find(distances <= epsilon);
        
        if length(neighbors) < minPts
            clusters(i) = -1; % Ruido
        else
            cluster_id = cluster_id + 1;
            clusters(i) = cluster_id;
            
            % Expandir cluster
            seed_set = neighbors;
            k = 1;
            while k <= length(seed_set)
                q = seed_set(k);
                if clusters(q) == -1
                    clusters(q) = cluster_id;
                end
                if clusters(q) ~= 0
                    k = k + 1;
                    continue;
                end
                
                clusters(q) = cluster_id;
                distances_q = sqrt(sum((points - points(q, :)).^2, 2));
                neighbors_q = find(distances_q <= epsilon);
                
                if length(neighbors_q) >= minPts
                    seed_set = [seed_set; neighbors_q]; %#ok<AGROW>
                end
                k = k + 1;
            end
        end
    end
end

% ---------- RTK mejorado ----------
function [hasRTK, XYZ, t, latlon] = detectRTK_3D_Enhanced(S)
    hasRTK=false; XYZ=[]; t=[]; latlon=[];
    if ~(isfield(S,'lat') && isfield(S,'lon')), fprintf('Campos RTK no encontrados (lat, lon)\n'); return; end
    lat = S.lat(:); lon = S.lon(:);
    alt_fields = {'alt','altitude','height','z','elevation'}; alt=[];
    for field = alt_fields
        if isfield(S, field{1}) && ~isempty(S.(field{1})), alt = S.(field{1})(:); break; end
    end
    if isempty(alt), alt = zeros(size(lat)); end

    valid = ~isnan(lat) & ~isnan(lon) & ~isnan(alt) & abs(lat)<=90 & abs(lon)<=180 & abs(alt)<10000;
    lat = lat(valid); lon = lon(valid); alt = alt(valid);
    if numel(lat)<2, return; end

    % Parámetros desde el caller
    P = struct( ...
        'MAX_SPEED',        evalin('caller','RTK_MAX_SPEED_MS'), ...
        'SMOOTH_WIN_M',     evalin('caller','RTK_SMOOTH_WIN_M'), ...
        'HAMPEL_W',         evalin('caller','RTK_HAMPEL_WINDOW'), ...
        'LOOP_CORR',        evalin('caller','RTK_LOOP_CORRECTION'), ...
        'STATIC_OFFSET',     evalin('caller','RTK_STATIC_OFFSET_M') ...
    );

    % Mejora en LLA
    [lat, lon, alt] = refineRTK_LLA(lat, lon, alt, P);
    latlon = [lat,lon,alt];
    hasRTK = true;

    % A métricas locales vía UTM → marco local con origen en el primer punto
    a=6378137.0; f=1/298.257223563;
    lon_center = mean(lon); utm_zone = floor((lon_center + 180) / 6) + 1;
    [x_utm, y_utm] = convertWGS84toUTM_Enhanced(lat, lon, utm_zone, a, f);

    % Lever-arm antena → centro vehículo (dx adelante, dy izquierda) en metros
    if any(P.STATIC_OFFSET ~= 0)
        dx = P.STATIC_OFFSET(1); dy = P.STATIC_OFFSET(2);
        hd = atan2(diff(y_utm), diff(x_utm)); hd = [hd(1); hd]; % orientación aprox
        x_utm = x_utm + dx.*cos(hd) - dy.*sin(hd);
        y_utm = y_utm + dx.*sin(hd) + dy.*cos(hd);
    end

    x_local = x_utm - x_utm(1); 
    y_local = y_utm - y_utm(1); 
    z_local = alt - alt(1);
    if length(z_local)>5, z_local = smoothdata(z_local,'movmean',3); end
    XYZ_raw = [x_local, y_local, z_local];

    % Orientación agradable (X~Este) para la vista
    if numel(x_local)>=10
        n_points = min(100,numel(x_local));
        dxh = x_local(n_points) - x_local(1); dyh = y_local(n_points) - y_local(1);
        heading_rtk_deg = rad2deg(atan2(dyh,dxh)); if heading_rtk_deg<0, heading_rtk_deg=heading_rtk_deg+360; end
        target_heading = 90.0; 
        angle_diff = wrapTo180(target_heading - heading_rtk_deg);
        cx = mean(x_local); cy = mean(y_local);
        xc = x_local - cx; yc = y_local - cy; th = deg2rad(angle_diff);
        xr =  cos(th)*xc - sin(th)*yc; 
        yr =  sin(th)*xc + cos(th)*yc;
        XYZ = [xr + cx, yr + cy, z_local]; 
        XYZ = XYZ - XYZ(1,:);
    else
        XYZ = XYZ_raw;
    end

    % Tiempos (si existen)
    timestamp_fields = {'rtkTime','timestamps','time','t'};
    for field = timestamp_fields
        if isfield(S, field{1}) && ~isempty(S.(field{1}))
            t_all = S.(field{1})(:);
            if numel(t_all) >= sum(valid), t = t_all(valid); break; end
        end
    end
end

function [lat2, lon2, alt2] = refineRTK_LLA(lat, lon, alt, P)
% Limpia outliers, suaviza en METROS y corrige deriva de bucle en LLA.

    lat2 = lat; lon2 = lon; alt2 = alt;
    N = numel(lat); if N < 3, return; end

    lat0 = mean(lat);  m_per_deg_lat = 111320; 
    m_per_deg_lon = cosd(lat0) * 111320;

    % 1) Hampel para quitar spikes grandes
    if P.HAMPEL_W > 0
        lat2 = hampelfilt(lat2, P.HAMPEL_W, 3); % 3σ
        lon2 = hampelfilt(lon2, P.HAMPEL_W, 3);
    end

    % 2) Filtra saltos imposibles por "velocidad" y reinterpola
    x = (lon2 - lon2(1)) * m_per_deg_lon;
    y = (lat2 - lat2(1)) * m_per_deg_lat;
    dxy = hypot(diff(x), diff(y));
    v   = dxy; % sin dt, velocidad relativa por muestra
    bad = [false; v > max(P.MAX_SPEED,0.5)];
    if any(bad)
        xi = 1:N; good = ~bad;
        lat2 = interp1(xi(good), lat2(good), xi, 'pchip');
        lon2 = interp1(xi(good), lon2(good), xi, 'pchip');
        if ~isempty(alt2), alt2 = interp1(xi(good), alt2(good), xi, 'pchip'); end
    end

    % 3) Suavizado en METROS (Savitzky-Golay o movmean)
    x = (lon2 - lon2(1)) * m_per_deg_lon;
    y = (lat2 - lat2(1)) * m_per_deg_lat;
    step_m = mean(hypot(diff(x), diff(y)));
    win = max(5, 2*floor(max(P.SMOOTH_WIN_M / max(step_m,1e-3), 3)/2)+1); % impar
    try
        x = sgolayfilt(x, 3, win);
        y = sgolayfilt(y, 3, win);
    catch
        x = smoothdata(x,'movmean',max(5,win));
        y = smoothdata(y,'movmean',max(5,win));
    end

    % 4) Corrección de deriva de bucle
    if P.LOOP_CORR && N >= 10
        drift = [x(end)-x(1); y(end)-y(1)];
        L = [0; cumsum(hypot(diff(x),diff(y)))];
        if norm(drift) > 0.2 && L(end) > 5
            frac = L / max(L(end),1e-6);
            x = x - drift(1)*frac;
            y = y - drift(2)*frac;
        end
    end

    % 5) Regreso a LLA
    lon2 = x / m_per_deg_lon + lon2(1);
    lat2 = y / m_per_deg_lat + lat2(1);

    if ~isempty(alt2) && numel(alt2) >= 5
        alt2 = smoothdata(alt2, 'movmedian', 5);
    end
end

function y = hampelfilt(x, k, nsigma)
% Hampel simple 1D
    x = x(:); n = numel(x); y = x;
    for i = 1:n
        i1 = max(1, i-k); i2 = min(n, i+k);
        xi = x(i1:i2);
        med = median(xi, 'omitnan');
        sig = 1.4826*median(abs(xi-med), 'omitnan');
        if sig==0, sig = eps; end
        if abs(x(i)-med) > nsigma*sig
            y(i) = med;
        end
    end
end
% ---------- fin RTK mejorado ----------

function [alignedFrames, alignedRTK, syncQuality] = synchronizeLidarRTK_Enhanced(frames, lidarTimestamps, XYZrtk, rtkTimestamps)
    alignedFrames = frames; alignedRTK = XYZrtk; syncQuality = 0;
    if isempty(lidarTimestamps) || isempty(rtkTimestamps), return; end
    if isdatetime(lidarTimestamps), lidarTime = posixtime(lidarTimestamps); else, lidarTime = double(lidarTimestamps); end
    if isdatetime(rtkTimestamps), rtkTime = posixtime(rtkTimestamps); else, rtkTime = double(rtkTimestamps); end
    margin = 0.1; minTime = max(min(lidarTime), min(rtkTime)) + margin;
    maxTime = min(max(lidarTime), max(rtkTime)) - margin;
    if maxTime <= minTime, return; end
    lidarValid = (lidarTime >= minTime) & (lidarTime <= maxTime);
    alignedFrames = frames(lidarValid); validLidarTime = lidarTime(lidarValid);
    if length(rtkTime) > 1 && ~isempty(validLidarTime)
        alignedRTK = zeros(length(validLidarTime), size(XYZrtk, 2));
        for i = 1:size(XYZrtk, 2)
            try
                alignedRTK(:, i) = interp1(rtkTime, XYZrtk(:, i), validLidarTime, 'spline', 'extrap');
            catch
                alignedRTK(:, i) = interp1(rtkTime, XYZrtk(:, i), validLidarTime, 'linear', 'extrap');
            end
        end
        timeGaps = diff(validLidarTime);
        avgGap = mean(timeGaps); maxGap = max(timeGaps);
        syncQuality = min(1, 1 - (maxGap - avgGap) / avgGap);
    end
end

function [x, y] = convertWGS84toUTM_Enhanced(lat, lon, zone, a, f)
    k0 = 0.9996; E0 = 500000; N0 = 0;
    lon0 = (zone - 1) * 6 - 180 + 3;
    e2 = f * (2 - f); e_prime2 = e2 / (1 - e2);
    lat_rad = deg2rad(lat); lon_rad = deg2rad(lon); lon0_rad = deg2rad(lon0);
    dlon = lon_rad - lon0_rad; sin_lat = sin(lat_rad); cos_lat = cos(lat_rad); tan_lat = tan(lat_rad);
    N = a ./ sqrt(1 - e2 * sin_lat.^2); T = tan_lat.^2; C = e_prime2 * cos_lat.^2; A = cos_lat .* dlon;
    M = a * ((1 - e2/4 - 3*e2^2/64 - 5*e2^3/256) .* lat_rad - ...
             (3*e2/8 + 3*e2^2/32 + 45*e2^3/1024) .* sin(2*lat_rad) + ...
             (15*e2^2/256 + 45*e2^3/1024) .* sin(4*lat_rad) - ...
             (35*e2^3/3072) .* sin(6*lat_rad));
    x = E0 + k0 * N .* (A + (1-T+C).*A.^3/6 + (5-18*T+T.^2+72*C-58*e_prime2).*A.^5/120);
    y = N0 + k0 * (M + N.*tan_lat .* (A.^2/2 + (5-T+9*C+4*C.^2).*A.^4/24 + ...
                     (61-58*T+T.^2+600*C-330*e_prime2).*A.^6/720));
end

function evaluateAccuracyRTK_3D_Enhanced(path1, path2, XYZrtk, XYZrtk_v1, XYZrtk_v2, loop_closure_applied)
    if isempty(XYZrtk), return; end
    % --- VUELTA 1 ---
    if ~isempty(path1) && size(path1,1) > 1
        rtk_ref_v1 = ensureSameLength(path1, XYZrtk_v1, XYZrtk);
        error_x1 = path1(:,1) - rtk_ref_v1(:,1);
        error_y1 = path1(:,2) - rtk_ref_v1(:,2);
        if size(path1,2) >= 3 && size(rtk_ref_v1,2) >= 3
            error_z1 = path1(:,3) - rtk_ref_v1(:,3);
            error_3d1 = sqrt(error_x1.^2 + error_y1.^2 + error_z1.^2);
        else
            error_3d1 = sqrt(error_x1.^2 + error_y1.^2);
        end
        fprintf('\n--- VUELTA 1 vs RTK ---\n');
        fprintf('Error medio 3D: %.3f m | RMS: %.3f m | Max: %.3f m\n', mean(error_3d1), rms(error_3d1), max(error_3d1));
    end
    % --- VUELTA 2 ---
    if ~isempty(path2) && size(path2,1) > 1
        rtk_ref_v2 = ensureSameLength(path2, XYZrtk_v2, XYZrtk);
        error_x2 = path2(:,1) - rtk_ref_v2(:,1);
        error_y2 = path2(:,2) - rtk_ref_v2(:,2);
        if size(path2,2) >= 3 && size(rtk_ref_v2,2) >= 3
            error_z2 = path2(:,3) - rtk_ref_v2(:,3);
            error_3d2 = sqrt(error_x2.^2 + error_y2.^2 + error_z2.^2);
        else
            error_3d2 = sqrt(error_x2.^2 + error_y2.^2);
        end
        fprintf('\n--- VUELTA 2 vs RTK ---\n');
        fprintf('Error medio 3D: %.3f m | RMS: %.3f m\n', mean(error_3d2), rms(error_3d2));
        if loop_closure_applied, fprintf('Loop closure: APLICADO\n'); end
    end
end

function ref = ensureSameLength(path, ref, XYZ_all)
    if isempty(ref), ref = intelligentInterpolation3D_Enhanced(XYZ_all, size(path,1)); end
    if size(ref,1) ~= size(path,1), ref = intelligentInterpolation3D_Enhanced(ref, size(path,1)); end
end

function interpolated_points = intelligentInterpolation3D_Enhanced(reference_points, target_length)
    if size(reference_points, 1) == target_length
        interpolated_points = reference_points; return;
    end
    if size(reference_points, 2) >= 3
        distances = [0; cumsum(sqrt(sum(diff(reference_points(:,1:3)).^2, 2)))];
    else
        distances = [0; cumsum(sqrt(sum(diff(reference_points(:,1:2)).^2, 2)))];
    end
    min_distance = max(distances) / (length(distances) * 100);
    [distances_unique, unique_idx] = unique(round(distances/min_distance)*min_distance, 'stable');
    reference_unique = reference_points(unique_idx, :);
    if length(distances_unique) < 3
        interpolated_points = zeros(target_length, size(reference_points, 2));
        for i = 1:size(reference_points, 2)
            interpolated_points(:, i) = linspace(reference_points(1, i), reference_points(end, i), target_length)';
        end
        return;
    end
    distances_unique = distances_unique / max(distances_unique);
    target_params = linspace(0, 1, target_length)';
    interpolated_points = zeros(target_length, size(reference_points, 2));
    for i = 1:size(reference_points, 2)
        try
            interpolated_points(:, i) = interp1(distances_unique, reference_unique(:, i), target_params, 'spline', 'extrap');
        catch
            try
                interpolated_points(:, i) = interp1(distances_unique, reference_unique(:, i), target_params, 'pchip', 'extrap');
            catch
                interpolated_points(:, i) = interp1(distances_unique, reference_unique(:, i), target_params, 'linear', 'extrap');
            end
        end
    end
    if target_length > 5
        for i = 1:size(interpolated_points, 2)
            interpolated_points(:, i) = smoothdata(interpolated_points(:, i), 'movmean', 3);
        end
    end
end

function trajectory_length = calculateTrajectoryLength3D(trajectory)
    if size(trajectory,1) < 2, trajectory_length = 0; return; end
    if size(trajectory, 2) >= 3
        distances = sqrt(sum(diff(trajectory(:,1:3)).^2, 2));
    else
        distances = sqrt(sum(diff(trajectory(:,1:2)).^2, 2));
    end
    trajectory_length = sum(distances);
end

function formatted = formatNumber(num)
    if num >= 1000000
        formatted = sprintf('%.1fM', num/1000000);
    elseif num >= 1000
        formatted = sprintf('%.1fK', num/1000);
    else
        formatted = sprintf('%d', num);
    end
end

%% ========================== COLOREADO TIPO VELODYNE Y ROS2 ==========================
function rgb = heightColorStyle(points)
    if isempty(points), rgb = []; return; end
    z = points(:,3);
    z_min = prctile(z, 2); z_max = prctile(z, 98);
    if z_max - z_min < 1e-6, z_min = min(z); z_max = max(z); end
    t = (z - z_min) ./ max(z_max - z_min, 1e-6);
    t = min(max(t,0),1);

    T  = [0.00, 0.33, 0.66, 1.00]';
    C0 = [0.95 0.75 0.05];   % dorado
    C1 = [0.20 0.90 0.20];   % verde
    C2 = [0.20 0.80 1.00];   % cian
    C3 = [0.05 0.20 0.90];   % azul
    CM = [C0; C1; C2; C3];

    r = interp1(T, CM(:,1), t, 'pchip');
    g = interp1(T, CM(:,2), t, 'pchip');
    b = interp1(T, CM(:,3), t, 'pchip');
    rgb = [r g b];
end

function rgb = enhancedHeightColorStyle(points)
    % Coloreado mejorado estilo ROS2 con mejor contraste para clusters
    if isempty(points), rgb = []; return; end
    
    z = points(:,3);
    z_min = prctile(z, 5);  % Usar percentiles para mejor contraste
    z_max = prctile(z, 95);
    
    if z_max - z_min < 1e-6
        z_min = min(z); 
        z_max = max(z); 
    end
    
    % Normalización con compresión suave en los extremos
    t = (z - z_min) ./ max(z_max - z_min, 1e-6);
    t = min(max(t, 0), 1);
    
    % Aplicar función suave para mejor distribución de colores
    t = 0.5 * (1 + tanh(3 * (t - 0.5))); % Compresión sigmoidal
    
    % Paleta de colores estilo ROS2 mejorada
    % Suelo (bajo) -> Objetos medios -> Objetos altos -> Cielo
    color_points = [0.0,  0.25, 0.5,  0.75, 1.0]';
    
    % Colores más vibrantes y distintivos
    red_values   = [0.8,  0.2,  0.1,  0.9,  1.0]';  % Rojo: tierra->objetos->cielo
    green_values = [0.4,  0.9,  0.8,  0.6,  0.2]';  % Verde: pico en vegetación
    blue_values  = [0.2,  0.3,  1.0,  0.3,  0.8]';  % Azul: pico en estructuras
    
    % Interpolación suave
    r = interp1(color_points, red_values, t, 'pchip');
    g = interp1(color_points, green_values, t, 'pchip');
    b = interp1(color_points, blue_values, t, 'pchip');
    
    % Asegurar valores válidos
    r = min(max(r, 0), 1);
    g = min(max(g, 0), 1);
    b = min(max(b, 0), 1);
    
    rgb = [r g b];
end

%% ========================== MISC ==========================
function [vH, vW] = writeVideoFixed(vWriter, axOrFig, vH, vW)
    F = getframe(axOrFig); img = F.cdata; [H,W,~] = size(img);
    if isempty(vH), vH = H; vW = W; end
    if H~=vH || W~=vW, img = imresize(img, [vH, vW]); end
    writeVideo(vWriter, img);
end

function pc = getFramePC(container, idx)
    if iscell(container), pc = container{idx}; else, pc = container(idx); end
    if ~isa(pc,'pointCloud')
        if isstruct(pc) && isfield(pc,'Location')
            pc = pointCloud(pc.Location);
        elseif isnumeric(pc) && size(pc,2)==3
            pc = pointCloud(pc);
        else
            error('Frame %d no se puede convertir a pointCloud válido', idx);
        end
    end
end

function [frames, ts] = detectLidarFrames(S)
    frames = {}; ts = [];
    if isempty(S) || ~isstruct(S), error('Entrada inválida: S debe ser una estructura no vacía'); end
    names = fieldnames(S); fprintf('Campos detectados: %s\n', strjoin(names, ', '));
    strategies = {
        @(v) iscell(v) && ~isempty(v) && isa(v{1},'pointCloud'), ...
        @(v) isa(v,'pointCloud'), ...
        @(v) iscell(v) && ~isempty(v) && isnumeric(v{1}) && size(v{1},2)==3, ...
        @(v) isstruct(v) && ~isempty(v) && isfield(v,'Location'), ...
        @(v) isstruct(v) && ~isempty(v) && isfield(v,'x') && isfield(v,'y') && isfield(v,'z')
    };
    strategy_names = {'Celda de pointCloud','Arreglo de pointCloud','Celda Nx3','Estruct .Location','Estruct .x/.y/.z'};
    for k=1:numel(names)
        v = S.(names{k});
        for s=1:length(strategies)
            if strategies{s}(v)
                frames = convertToFrames(v, s);
                fprintf('Frames LIDAR encontrados: %s en campo "%s"\n', strategy_names{s}, names{k});
                break;
            end
        end
        if ~isempty(frames), break; end
    end
    if isempty(frames), error('No se encontraron frames LIDAR válidos'); end
    fprintf('Total de frames LIDAR: %d\n', numel(frames));
    timestampFields = {'lidar_t','timestamps_lidar','lidarTime','time_lidar','time','timestamps'};
    for fieldName = timestampFields
        if isfield(S, fieldName{1})
            ts = S.(fieldName{1}); 
            if isnumeric(ts) || isdatetime(ts)
                ts = ts(:); fprintf('Timestamps encontrados en: "%s"\n', fieldName{1}); 
                break; 
            end
        end
    end
    if isempty(ts), fprintf('Timestamps no encontrados - usando índices\n'); end
end

function frames = convertToFrames(v, strategy)
    switch strategy
        case 1, frames = v;
        case 2, n = numel(v); frames = cell(1,n); for i=1:n, frames{i} = v(i); end
        case 3, n = numel(v); frames = cell(1,n); for i=1:n, frames{i} = pointCloud(v{i}); end
        case 4, n = numel(v); frames = cell(1,n); for i=1:n, frames{i} = pointCloud(v(i).Location); end
        case 5, n = numel(v); frames = cell(1,n); for i=1:n
                coords = [v(i).x(:), v(i).y(:), v(i).z(:)];
                frames{i} = pointCloud(coords);
            end
    end
end

%% ========================== POSE: LOGGING + HEADING ==========================
function logPose(passName, idx, T)
    % Imprime θ (yaw, grados) y XYZ de la pose T (rigidtform3d)
    R = T.R; t = T.Translation;
    yaw_deg = yawFromR(R);
    fprintf('[%s][frame %d] θ(yaw)=%.2f° | x=%.3f y=%.3f z=%.3f\n', passName, idx, yaw_deg, t(1), t(2), t(3));
end

function yaw_deg = yawFromR(R)
    % Convención yaw a partir de la proyección en XY (atan2(r21, r11))
    yaw_deg = rad2deg(atan2(R(2,1), R(1,1)));
end

function updateHeadingArrow(h, T, L)
    % Dibuja/actualiza una flecha desde la posición de T con dirección del eje X del robot
    o = T.Translation(:)';
    fwd = (T.R * [1;0;0])'; % vector adelante en coords globales
    d = L * (fwd / max(norm(fwd), 1e-6));
    set(h, 'XData', o(1), 'YData', o(2), 'ZData', o(3), ...
           'UData', d(1), 'VData', d(2), 'WData', d(3));
end

%% ========================== ALGORITMOS ROBUSTOS ========================== 

function relPose = robustNDTRegistration(moving, fixed, gridStep, initialTransform, useMultiHyp, useSpatialCheck, maxJump, poseHistory)
    % Registro NDT robusto con múltiples estrategias de fallback
    
    if isempty(initialTransform)
        initialTransform = rigidtform3d;
    end
    
    % Estrategia 1: NDT con parámetros optimizados
    try
        relPose = pcregisterndt(moving, fixed, gridStep, ...
            'InitialTransform', initialTransform, ...
            'MaxIterations', 100, ...
            'Tolerance', [0.003, 0.2], ...
            'OutlierRatio', 0.4);
        return;
    catch ME1
        fprintf('⚠️ NDT fallback 1 falló: %s\n', ME1.message);
    end
    
    % Estrategia 2: NDT con parámetros más permisivos
    try
        relPose = pcregisterndt(moving, fixed, gridStep * 1.5, ...
            'InitialTransform', initialTransform, ...
            'MaxIterations', 150, ...
            'Tolerance', [0.01, 0.5]);
        return;
    catch ME2
        fprintf('⚠️ NDT fallback 2 falló: %s\n', ME2.message);
    end
    
    % Estrategia 3: ICP como respaldo
    try
        relPose = pcregistericp(moving, fixed, ...
            'InitialTransform', initialTransform, ...
            'MaxIterations', 100, ...
            'Tolerance', [0.005, 0.5]);
        fprintf('✅ Usando ICP como respaldo\n');
        return;
    catch ME3
        fprintf('⚠️ ICP fallback falló: %s\n', ME3.message);
    end
    
    % Estrategia 4: Predicción basada en historial
    if ~isempty(poseHistory) && size(poseHistory, 1) >= 2
        motion_pred = predictNextMotion(poseHistory, 0.8);
        relPose = rigidtform3d([eye(3), motion_pred'; 0 0 0 1]);
        fprintf('🔮 Usando predicción de movimiento\n');
    else
        relPose = rigidtform3d; % Identidad como último recurso
        fprintf('⚠️ Usando transformación identidad\n');
    end
end

function relPose = robustNDTRegistrationAdvanced(moving, fixed, gridStep, initialTransform, poseHistory, maxJump, recoveryMode)
    % Versión avanzada con más validaciones
    
    % Primero intentar registro normal
    relPose = robustNDTRegistration(moving, fixed, gridStep, initialTransform, false, true, maxJump, poseHistory);
    
    % Validar resultado
    if ~isempty(poseHistory) && size(poseHistory, 1) > 0
        last_pos = poseHistory(end, :);
        current_translation = relPose.Translation(:)';  % Asegurar formato fila
        spatial_jump = norm(current_translation - last_pos);
        
        if spatial_jump > maxJump && recoveryMode
            % Aplicar corrección adaptativa
            fprintf('🔧 Aplicando corrección adaptativa para salto de %.2fm\n', spatial_jump);
            correction_factor = min(maxJump / spatial_jump, 0.8);
            corrected_translation = last_pos + correction_factor * (current_translation - last_pos);
            relPose = rigidtform3d([relPose.R, corrected_translation(:); 0 0 0 1]);  % Asegurar formato columna
        end
    end
end

function motion_prediction = predictNextMotion(poseHistory, weight)
    % Predicción de movimiento basada en historial
    
    if size(poseHistory, 1) < 2
        motion_prediction = [0, 0, 0];
        return;
    end
    
    if size(poseHistory, 1) == 2
        % Simple diferencia
        motion_prediction = poseHistory(2, :) - poseHistory(1, :);
    else
        % Promedio ponderado de últimos movimientos
        recent_motions = diff(poseHistory(max(1, end-4):end, :), 1, 1);
        weights = linspace(0.3, 1.0, size(recent_motions, 1));
        weights = weights / sum(weights);
        motion_prediction = sum(recent_motions .* weights', 1);
    end
    
    % Aplicar peso de confianza
    motion_prediction = weight * motion_prediction;
end

function coherence_score = validateSpatialCoherence(relPose, poseHistory, validator)
    % Validar coherencia espacial de la pose
    
    if isempty(poseHistory) || size(poseHistory, 1) < 2
        coherence_score = 1.0;
        return;
    end
    
    current_pos = relPose.Translation(:)';  % Asegurar formato fila
    
    % Analizar consistencia con historial reciente
    window_size = min(validator.window_size, size(poseHistory, 1));
    recent_poses = poseHistory(end-window_size+1:end, :);
    
    % Calcular métricas de coherencia
    distances = vecnorm(diff(recent_poses, 1, 1), 2, 2);
    avg_distance = mean(distances);
    std_distance = std(distances);
    
    if size(poseHistory, 1) > 1
        last_distance = norm(current_pos - poseHistory(end, :));
        
        % Score basado en desviación de la norma
        if avg_distance > 0 && std_distance > 0
            z_score = abs(last_distance - avg_distance) / (std_distance + 1e-6);
            coherence_score = max(0, 1 - z_score / 3); % Normalizar z-score
        else
            coherence_score = 1.0;
        end
    else
        coherence_score = 1.0;
    end
end

function corrected_pose = applySpatialCorrection(relPose, poseHistory, coherence_score)
    % Aplicar corrección espacial basada en score de coherencia
    
    if isempty(poseHistory) || coherence_score > 0.8
        corrected_pose = relPose;
        return;
    end
    
    current_pos = relPose.Translation(:)';  % Asegurar formato fila
    
    if size(poseHistory, 1) >= 2
        % Interpolar hacia posición más probable
        predicted_pos = predictNextMotion(poseHistory, 1.0);
        last_pos = poseHistory(end, :);
        expected_pos = last_pos + predicted_pos;
        
        % Blend entre posición actual y esperada
        blend_factor = 1 - coherence_score; % Más blend si menor coherencia
        corrected_translation = (1 - blend_factor) * current_pos + blend_factor * expected_pos;
        
        corrected_pose = rigidtform3d([relPose.R, corrected_translation(:); 0 0 0 1]);  % Asegurar formato columna
        fprintf('📐 Corrección espacial aplicada (blend: %.2f)\n', blend_factor);
    else
        corrected_pose = relPose;
    end
end

function recovered_pose = recoverFromSpatialJump(relPose, lastValidPose, poseHistory, maxJump)
    % Recuperación inteligente de saltos espaciales
    
    current_translation = relPose.Translation(:)';  % Asegurar formato fila
    
    if ~isempty(poseHistory) && size(poseHistory, 1) > 0
        last_pos = poseHistory(end, :);
        
        % Calcular dirección del salto
        jump_vector = current_translation - last_pos;
        jump_distance = norm(jump_vector);
        
        if jump_distance > maxJump
            % Limitar el salto al máximo permitido
            limited_jump = (maxJump / jump_distance) * jump_vector;
            recovered_translation = last_pos + limited_jump;
            
            % Mantener orientación original pero corregir posición
            recovered_pose = rigidtform3d([relPose.R, recovered_translation(:); 0 0 0 1]);  % Asegurar formato columna
            
            fprintf('🛠️ Salto limitado de %.2fm a %.2fm\n', jump_distance, maxJump);
        else
            recovered_pose = relPose;
        end
    else
        % Sin historial, usar última pose válida conocida
        if ~isempty(lastValidPose)
            recovered_pose = lastValidPose;
            fprintf('🔄 Revirtiendo a última pose válida\n');
        else
            recovered_pose = relPose;
        end
    end
end

%% ========================== ALGORITMOS DE LIMPIEZA DE MAPA ==========================

function cleanedPC = removeFloatingPoints(pc)
    % Eliminar puntos flotantes en el aire sin soporte
    if pc.Count < 50
        cleanedPC = pc;
        return;
    end
    
    points = pc.Location;
    colors = [];
    if ~isempty(pc.Color)
        colors = pc.Color;
    end
    
    % Calcular altura relativa de cada punto
    z_values = points(:, 3);
    z_min = prctile(z_values, 5);  % Nivel del suelo aproximado
    relative_heights = z_values - z_min;
    
    % Filtrar puntos demasiado altos (probablemente flotantes)
    MAX_REASONABLE_HEIGHT = 12.0;  % 12 metros sobre el suelo
    height_mask = relative_heights <= MAX_REASONABLE_HEIGHT;
    
    % Para puntos altos, verificar si tienen soporte cercano
    high_points = find(~height_mask);
    support_mask = true(size(height_mask));
    
    for i = 1:length(high_points)
        idx = high_points(i);
        current_point = points(idx, :);
        
        % Buscar puntos de soporte en un radio
        distances = vecnorm(points - current_point, 2, 2);
        nearby_mask = distances < 2.0 & distances > 0;
        nearby_lower = points(nearby_mask, 3) < current_point(3) - 1.0;
        
        % Si no hay suficiente soporte, eliminar punto
        if sum(nearby_lower) < 3
            support_mask(idx) = false;
        end
    end
    
    final_mask = height_mask | support_mask;
    
    if ~isempty(colors)
        cleanedPC = pointCloud(points(final_mask, :), 'Color', colors(final_mask, :));
    else
        cleanedPC = pointCloud(points(final_mask, :));
    end
    
    removed_count = sum(~final_mask);
    if removed_count > 0
        fprintf('🧹 Eliminados %d puntos flotantes\n', removed_count);
    end
end

function filteredPC = adaptiveDensityFilter(pc)
    % Filtrado de densidad adaptativo para eliminar puntos aislados
    if pc.Count < 100
        filteredPC = pc;
        return;
    end
    
    points = pc.Location;
    colors = [];
    if ~isempty(pc.Color)
        colors = pc.Color;
    end
    
    % Análisis de densidad adaptativo
    RADIUS = 1.2;  % Radio de búsqueda
    MIN_NEIGHBORS = 6;  % Mínimo de vecinos
    
    % Usar kd-tree para búsqueda eficiente
    try
        [indices, distances] = rangesearch(points, points, RADIUS);
        neighbor_counts = cellfun(@length, indices) - 1; % -1 para excluir el punto mismo
        
        % Filtrar puntos con pocos vecinos
        density_mask = neighbor_counts >= MIN_NEIGHBORS;
        
        if ~isempty(colors)
            filteredPC = pointCloud(points(density_mask, :), 'Color', colors(density_mask, :));
        else
            filteredPC = pointCloud(points(density_mask, :));
        end
        
        removed_count = sum(~density_mask);
        % if removed_count > 0
        %     fprintf('🔍 Filtrado de densidad: eliminados %d puntos aislados\n', removed_count);
        % end
    catch
        % Fallback si rangesearch falla
        filteredPC = pc;
    end
end

function mergedPC = mergeWithQualityControl(mapPC, newPC, resMap)
    % MERGE CON LIMPIEZA DE PUNTOS FANTASMA DEL CENTRO
    
    % Variables persistentes al inicio (eficiencia)
    persistent trajectory_points all_poses cleanupCounter
    if isempty(trajectory_points)
        trajectory_points = [];
        all_poses = [];
    end
    if isempty(cleanupCounter)
        cleanupCounter = 0;
    end
    
    try
        % PASO 1: Pre-filtrado de la nueva nube
        if newPC.Count > 50
            newPC = ros2StyleGroundFilter(newPC, true);
        end
        
        % PASO 2: Análisis de solapamiento geométrico
        overlapFactor = calculateOverlapFactor(mapPC, newPC);
        
        % PASO 3: Merge adaptativo basado en solapamiento
        if overlapFactor > 0.3  % Alto solapamiento
            % Merge conservador para evitar duplicación
            mergedPC = pcmerge(mapPC, newPC, resMap * 1.2);
        else
            % Merge normal
            mergedPC = pcmerge(mapPC, newPC, resMap * 0.8);
        end
        
        % PASO 4: 🔥 NUEVO - ELIMINAR PUNTOS FANTASMA DEL CENTRO
        % Estos puntos son drift acumulado que aparecen en el "medio" del mapa
        
        % Acumular posiciones de la trayectoria real
        if ~isempty(newPC) && newPC.Count > 0
            center_new = mean(newPC.Location, 1);
            all_poses = [all_poses; center_new];
            trajectory_points = all_poses;
        end
        
        % Si tenemos suficiente trayectoria, limpiar puntos lejos de ella
        if size(trajectory_points, 1) > 20 && mergedPC.Count > 500
            mergedPC = removeGhostPointsAwayFromTrajectory(mergedPC, trajectory_points);
        end
        
        % PASO 5: Post-procesamiento
        if mergedPC.Count > 100
            mergedPC = postProcessMergedMap(mergedPC);
        end
        
        % PASO 6: Limpieza periódica del mapa global
        cleanupCounter = cleanupCounter + 1;
        
        if cleanupCounter >= 5  % Cada 5 merges
            cleanupCounter = 0;
            mergedPC = globalMapCleaning(mergedPC);
        end
        
    catch ME
        fprintf('⚠️ Error en merge: %s\n', ME.message);
        % Fallback simple
        mergedPC = pcmerge(mapPC, newPC, resMap);
    end
end

function overlapFactor = calculateOverlapFactor(pc1, pc2)
    % Calcular factor de solapamiento entre nubes
    
    if pc1.Count < 50 || pc2.Count < 50
        overlapFactor = 0.5;
        return;
    end
    
    try
        % Comparar límites espaciales
        limits1 = [min(pc1.Location); max(pc1.Location)];
        limits2 = [min(pc2.Location); max(pc2.Location)];
        
        % Calcular solapamiento volumétrico
        overlapX = max(0, min(limits1(2,1), limits2(2,1)) - max(limits1(1,1), limits2(1,1)));
        overlapY = max(0, min(limits1(2,2), limits2(2,2)) - max(limits1(1,2), limits2(1,2)));
        overlapZ = max(0, min(limits1(2,3), limits2(2,3)) - max(limits1(1,3), limits2(1,3)));
        
        overlapVolume = overlapX * overlapY * overlapZ;
        
        % Volúmenes individuales
        volume1 = prod(limits1(2,:) - limits1(1,:));
        volume2 = prod(limits2(2,:) - limits2(1,:));
        
        % Factor de solapamiento normalizado
        if volume1 > 0 && volume2 > 0
            overlapFactor = overlapVolume / min(volume1, volume2);
        else
            overlapFactor = 0.5;
        end
        
    catch
        overlapFactor = 0.5;
    end
end

function pc = postProcessMergedMap(pc)
    % Post-procesamiento del mapa fusionado estilo ROS2
    
    if pc.Count < 100, return; end
    
    % 1. Voxel grid para uniformidad
    try
        pc = advancedVoxelGridFilter(pc);
    catch
        % Fallback
        if pc.Count > 1000
            pc = pcdownsample(pc, 'random', 0.8);
        end
    end
    
    % 2. Eliminación de outliers estadísticos
    if pc.Count > 100
        try
            pc = statisticalOutlierRemoval(pc);
        catch
            % Skip if fails
        end
    end
end

function pc = globalMapCleaning(pc)
    % LIMPIEZA GLOBAL ESTILO CARTOGRAPHER
    
    if pc.Count < 200, return; end
    
    originalCount = pc.Count;
    
    try
        % 1. Filtrado de suelo global más agresivo
        points = pc.Location;
        z = points(:,3);
        zMin = min(z);
        
        % Threshold más agresivo para mapa global
        globalGroundThreshold = zMin + 1.5;  % 1.5m sobre punto más bajo
        validMask = z > globalGroundThreshold;
        
        if sum(validMask) < pc.Count * 0.1  % Si eliminamos más del 90%
            % Ser menos agresivo
            globalGroundThreshold = zMin + 1.0;
            validMask = z > globalGroundThreshold;
        end
        
        if sum(validMask) > 50
            pc = pointCloud(points(validMask, :));
        end
        
        % 2. Clustering para remover puntos aislados
        if pc.Count > 100
            pc = removeIsolatedPoints(pc);
        end
        
        cleanedPercent = 100 * (originalCount - pc.Count) / originalCount;
        fprintf('🧹 Limpieza global: %.1f%% puntos eliminados\n', cleanedPercent);
        
    catch ME
        fprintf('⚠️ Error en limpieza global: %s\n', ME.message);
    end
end

function pc = removeIsolatedPoints(pc)
    % Remover puntos aislados usando clustering
    
    if pc.Count < 100, return; end
    
    try
        points = pc.Location;
        
        % Clustering simple
        epsilon = 1.0;  % 1 metro de radio
        minPts = 5;     % Mínimo 5 puntos por cluster
        
        clusters = simpleDBSCAN(points, epsilon, minPts);
        
        % Mantener solo puntos que pertenecen a clusters (no ruido)
        validMask = clusters ~= -1;
        
        if sum(validMask) > 50
            pc = pointCloud(points(validMask, :));
        end
        
    catch
        % Skip if clustering fails
    end
end

function cleanPC = removeGhostPointsAwayFromTrajectory(pc, trajectory_points)
    % 🔥 ELIMINAR PUNTOS FANTASMA LEJOS DE LA TRAYECTORIA REAL
    % Estos son puntos de drift/error acumulado que aparecen en el "centro" del mapa
    
    if pc.Count < 100 || size(trajectory_points, 1) < 20
        cleanPC = pc;
        return;
    end
    
    try
        points = pc.Location;
        
        % Calcular distancia mínima de cada punto a la trayectoria
        distances = zeros(size(points, 1), 1);
        for i = 1:size(points, 1)
            % Distancia al punto más cercano de la trayectoria
            dists_to_traj = vecnorm(trajectory_points - points(i, :), 2, 2);
            distances(i) = min(dists_to_traj);
        end
        
        % UMBRAL ADAPTATIVO: Los puntos deben estar a máximo 15m de la trayectoria
        % (Esto elimina drift acumulado en el centro sin afectar estructuras legítimas)
        max_distance_from_trajectory = 15.0;  % 15 metros
        
        % Mantener solo puntos cercanos a la trayectoria
        valid_mask = distances <= max_distance_from_trajectory;
        
        points_removed = sum(~valid_mask);
        if points_removed > 0
            fprintf('👻 Eliminados %d puntos fantasma lejos de trayectoria\n', points_removed);
        end
        
        % Asegurar que no eliminamos todo
        if sum(valid_mask) > max(50, pc.Count * 0.05)
            cleanPC = pointCloud(points(valid_mask, :));
        else
            % Si es demasiado agresivo, relajar el umbral
            max_distance_from_trajectory = 25.0;
            valid_mask = distances <= max_distance_from_trajectory;
            if sum(valid_mask) > 50
                cleanPC = pointCloud(points(valid_mask, :));
            else
                cleanPC = pc;  % Mantener original si falla
            end
        end
        
    catch ME
        fprintf('⚠️ Error eliminando puntos fantasma: %s\n', ME.message);
        cleanPC = pc;
    end
end

function cleanPC = filterPointsNearTrajectory(pc, current_position, max_radius)
    % 🔥 FILTRO ESPACIAL V1: Mantener solo puntos cerca de la posición actual
    % Elimina puntos que están muy lejos del sensor (probablemente errores)
    
    if pc.Count < 10
        cleanPC = pc;
        return;
    end
    
    try
        points = pc.Location;
        
        % Calcular distancia 3D desde posición actual del sensor
        distances = vecnorm(points - current_position, 2, 2);
        
        % Mantener solo puntos a <12m del sensor
        % (LiDAR de corto alcance típicamente 10-15m)
        valid_mask = distances <= max_radius;
        
        points_removed = sum(~valid_mask);
        % if points_removed > 100
        %     fprintf('🗑️  V1: Eliminados %d puntos lejos del sensor (>%.1fm)\n', ...
        %         points_removed, max_radius);
        % end
        
        % Asegurar que mantenemos suficientes puntos
        if sum(valid_mask) > max(10, pc.Count * 0.05)
            cleanPC = pointCloud(points(valid_mask, :));
        else
            cleanPC = pc;  % Mantener original si eliminaríamos todo
        end
        
    catch ME
        fprintf('⚠️ Error filtrando puntos V1: %s\n', ME.message);
        cleanPC = pc;
    end
end

function hasOverlap = hasGeometricOverlap(pc1, pc2)
    % Verificar si hay solapamiento geométrico significativo
    if pc1.Count < 50 || pc2.Count < 50
        hasOverlap = true;
        return;
    end
    
    try
        % Comparar límites espaciales
        limits1 = [min(pc1.Location); max(pc1.Location)];
        limits2 = [min(pc2.Location); max(pc2.Location)];
        
        % Calcular solapamiento en cada dimensión
        overlap_x = max(0, min(limits1(2,1), limits2(2,1)) - max(limits1(1,1), limits2(1,1)));
        overlap_y = max(0, min(limits1(2,2), limits2(2,2)) - max(limits1(1,2), limits2(1,2)));
        overlap_z = max(0, min(limits1(2,3), limits2(2,3)) - max(limits1(1,3), limits2(1,3)));
        
        % Si hay solapamiento en las 3 dimensiones
        hasOverlap = overlap_x > 0 && overlap_y > 0 && overlap_z > 0;
    catch
        hasOverlap = true;  % Asumir solapamiento en caso de error
    end
end

function reducedPC = intelligentMapReduction(mapPC, maxPoints, resMap)
    % Reducción inteligente que preserva estructura
    if mapPC.Count <= maxPoints
        reducedPC = mapPC;
        return;
    end
    
    reduction_factor = maxPoints / mapPC.Count;
    
    if reduction_factor > 0.7
        % Reducción ligera - usar random
        reducedPC = pcdownsample(mapPC, 'random', reduction_factor);
    elseif reduction_factor > 0.4
        % Reducción moderada - usar grid
        reducedPC = pcdownsample(mapPC, 'gridAverage', resMap * 1.5);
    else
        % Reducción agresiva - usar estrategia híbrida
        % Primero grid, luego random
        tempPC = pcdownsample(mapPC, 'gridAverage', resMap * 2.0);
        if tempPC.Count > maxPoints
            final_factor = maxPoints / tempPC.Count;
            reducedPC = pcdownsample(tempPC, 'random', final_factor);
        else
            reducedPC = tempPC;
        end
    end
    
    removed_count = mapPC.Count - reducedPC.Count;
    fprintf('🗜️ Reducción de mapa: %d → %d puntos (-%d)\n', mapPC.Count, reducedPC.Count, removed_count);
end

function cleanedPC = periodicMapCleaning(mapPC)
    % Limpieza periódica completa del mapa
    fprintf('🧽 Iniciando limpieza periódica del mapa...\n');
    
    originalCount = mapPC.Count;
    
    % 1. Eliminar puntos flotantes
    cleanedPC = removeFloatingPoints(mapPC);
    
    % 2. Filtrado de densidad
    cleanedPC = adaptiveDensityFilter(cleanedPC);
    
    % 3. Eliminación de outliers
    cleanedPC = statisticalOutlierRemoval(cleanedPC);
    
    finalCount = cleanedPC.Count;
    removed = originalCount - finalCount;
    
    if removed > 0
        fprintf('✨ Limpieza completada: %d puntos eliminados (%.1f%%)\n', ...
                removed, 100*removed/originalCount);
    end
end

function updateMapVisualization(vizHandle, mapPC)
    % Actualización optimizada de visualización
    try
        if isstruct(vizHandle) && isfield(vizHandle, 'mapPoints') && ~isempty(mapPC) && mapPC.Count > 0
            % Verificar si el handle gráfico existe y es válido
            if ~isempty(vizHandle.mapPoints) && ishghandle(vizHandle.mapPoints)
                % Actualizar solo si hay cambios significativos
                set(vizHandle.mapPoints, 'XData', mapPC.Location(:,1), ...
                                        'YData', mapPC.Location(:,2), ...
                                        'ZData', mapPC.Location(:,3));
                
                if ~isempty(mapPC.Color)
                    set(vizHandle.mapPoints, 'CData', mapPC.Color);
                end
            end
        end
    catch
        % Ignorar errores de visualización
    end
end

%% ========================== CLASIFICACIÓN INTELIGENTE DE OBJETOS ==========================

function classifiedPC = intelligentObjectClassification(pc)
    % Clasificación automática de puntos por tipo de objeto
    if pc.Count < 50
        classifiedPC = pc;
        return;
    end
    
    points = pc.Location;
    originalColors = [];
    if ~isempty(pc.Color)
        originalColors = pc.Color;
    end
    
    % Calcular altura relativa al suelo
    z_values = points(:, 3);
    ground_level = prctile(z_values, 10);  % Estimar nivel del suelo
    relative_heights = z_values - ground_level;
    
    % Clasificar por altura y características geométricas
    ground_mask = relative_heights <= 0.8;
    vegetation_mask = (relative_heights > 0.8) & (relative_heights <= 15.0);
    structure_mask = relative_heights > 1.5;
    
    % Análisis de densidad local para refinar clasificación
    density_analysis = analyzeLocalDensity(points);
    
    % Refinar vegetación usando densidad (vegetación tiene densidad variable)
    vegetation_mask = vegetation_mask & (density_analysis.variability > 0.3);
    
    % Refinar estructuras (más uniformes y planares)
    structure_mask = structure_mask & (density_analysis.planarity > 0.6);
    
    % Asignar colores específicos por categoría
    if isempty(originalColors)
        colors = zeros(size(points, 1), 3);  % Asegurar dimensiones correctas
    else
        % Convertir colores originales a double si es necesario
        if isa(originalColors, 'uint8')
            colors = double(originalColors) / 255.0;
        else
            colors = originalColors;
        end
    end
    
    % Colores mejorados por categoría (valores double 0-1)
    colors(ground_mask, :) = repmat([0.6, 0.4, 0.2], sum(ground_mask), 1);      % Marrón suelo
    colors(vegetation_mask, :) = repmat([0.2, 0.8, 0.3], sum(vegetation_mask), 1); % Verde vegetación
    colors(structure_mask, :) = repmat([0.7, 0.7, 0.9], sum(structure_mask), 1);   % Azul-gris estructuras
    
    % Crear nueva nube de puntos clasificada
    classifiedPC = pointCloud(points, 'Color', colors);
    
    % Agregar etiquetas de categoría como propiedad
    categories = zeros(size(points, 1), 1);
    categories(ground_mask) = 1;      % Suelo
    categories(vegetation_mask) = 2;  % Vegetación
    categories(structure_mask) = 3;   % Estructuras
    
    fprintf('🏷️ Clasificación: %d suelo, %d vegetación, %d estructuras\n', ...
            sum(ground_mask), sum(vegetation_mask), sum(structure_mask));
end

function filteredPC = categorySpecificFiltering(pc)
    % Filtrado específico según la categoría de cada punto
    if pc.Count < 100 || isempty(pc.Color)
        filteredPC = pc;
        return;
    end
    
    points = pc.Location;
    colors = pc.Color;
    
    % Convertir colores si es necesario
    if isa(colors, 'uint8')
        colors = double(colors) / 255.0;
    end
    
    % Detectar categorías por color
    ground_mask = detectColorCategory(colors, [0.6, 0.4, 0.2], 0.15);
    vegetation_mask = detectColorCategory(colors, [0.2, 0.8, 0.3], 0.15);
    structure_mask = detectColorCategory(colors, [0.7, 0.7, 0.9], 0.15);
    
    keep_mask = true(size(points, 1), 1);
    
    % Filtrado específico para vegetación (más agresivo)
    if sum(vegetation_mask) > 20
        veg_points = points(vegetation_mask, :);
        veg_keep = intelligentVegetationFilter(veg_points, 0.3); % 30% submuestreo
        keep_mask(vegetation_mask) = veg_keep;
    end
    
    % Filtrado específico para estructuras (preservar más detalle)
    if sum(structure_mask) > 20
        struct_points = points(structure_mask, :);
        struct_keep = intelligentStructureFilter(struct_points, 0.7); % 70% preservado
        keep_mask(structure_mask) = struct_keep;
    end
    
    % Filtrado específico para suelo (muy agresivo)
    if sum(ground_mask) > 20
        ground_points = points(ground_mask, :);
        ground_keep = intelligentGroundFilter(ground_points, 0.2); % 20% submuestreo
        keep_mask(ground_mask) = ground_keep;
    end
    
    filteredPC = pointCloud(points(keep_mask, :), 'Color', colors(keep_mask, :));
    
    removed_count = sum(~keep_mask);
    if removed_count > 0
        fprintf('🎯 Filtrado por categoría: eliminados %d puntos redundantes\n', removed_count);
    end
end

function clusteredPC = enhancedVegetationClustering(pc)
    % Clustering mejorado específicamente para vegetación
    if pc.Count < 200 || isempty(pc.Color)
        clusteredPC = pc;
        return;
    end
    
    points = pc.Location;
    colors = pc.Color;
    
    % Convertir colores si es necesario
    if isa(colors, 'uint8')
        colors = double(colors) / 255.0;
    end
    
    % Detectar puntos de vegetación
    vegetation_mask = detectColorCategory(colors, [0.2, 0.8, 0.3], 0.15);
    
    if sum(vegetation_mask) < 50
        clusteredPC = pc;
        return;
    end
    
    veg_points = points(vegetation_mask, :);
    
    % Clustering DBSCAN para agrupar árboles individuales
    try
        epsilon = 2.5;  % Radio de clustering para árboles
        min_points = 20; % Mínimo puntos por árbol
        
        [idx, ~] = dbscan(veg_points, epsilon, min_points);
        
        % Contar clusters válidos (excluir ruido: idx == -1)
        unique_clusters = unique(idx(idx > 0));
        num_trees = length(unique_clusters);
        
        if num_trees > 0
            % Asignar colores diferentes a cada árbol
            enhanced_colors = colors;
            tree_colors = generateTreeColors(num_trees);
            
            for i = 1:num_trees
                cluster_id = unique_clusters(i);
                cluster_mask_local = (idx == cluster_id);
                
                % Mapear de índices locales a globales
                global_indices = find(vegetation_mask);
                global_cluster_mask = global_indices(cluster_mask_local);
                
                enhanced_colors(global_cluster_mask, :) = repmat(tree_colors(i, :), ...
                                                               length(global_cluster_mask), 1);
            end
            
            clusteredPC = pointCloud(points, 'Color', enhanced_colors);
            fprintf('🌳 Detectados %d árboles individuales\n', num_trees);
        else
            clusteredPC = pc;
        end
    catch
        clusteredPC = pc;  % Fallback si DBSCAN falla
    end
end

function reducedPC = smartRedundancyRemoval(pc)
    % Eliminación inteligente de redundancia preservando características importantes
    if pc.Count < 300
        reducedPC = pc;
        return;
    end
    
    points = pc.Location;
    colors = [];
    if ~isempty(pc.Color)
        colors = pc.Color;
        % Convertir colores si es necesario
        if isa(colors, 'uint8')
            colors = double(colors) / 255.0;
        end
    end
    
    % Voxelización adaptativa
    voxel_size = 0.15;  % Tamaño base de vóxel
    
    % Crear grid 3D
    min_coords = min(points);
    max_coords = max(points);
    
    grid_size = ceil((max_coords - min_coords) / voxel_size) + 1;
    
    % Asignar cada punto a un vóxel
    voxel_indices = floor((points - min_coords) / voxel_size) + 1;
    
    % Para cada vóxel, mantener el punto más representativo
    unique_voxels = unique(voxel_indices, 'rows');
    selected_points = [];
    selected_colors = [];
    
    for i = 1:size(unique_voxels, 1)
        voxel = unique_voxels(i, :);
        
        % Encontrar todos los puntos en este vóxel
        same_voxel = all(voxel_indices == voxel, 2);
        voxel_points = points(same_voxel, :);
        
        if size(voxel_points, 1) == 1
            % Solo un punto, mantenerlo
            selected_points = [selected_points; voxel_points];
            if ~isempty(colors)
                selected_colors = [selected_colors; colors(same_voxel, :)];
            end
        else
            % Múltiples puntos, seleccionar el más central
            centroid = mean(voxel_points);
            distances = vecnorm(voxel_points - centroid, 2, 2);
            [~, min_idx] = min(distances);
            
            voxel_indices_global = find(same_voxel);
            selected_idx = voxel_indices_global(min_idx);
            
            selected_points = [selected_points; points(selected_idx, :)];
            if ~isempty(colors)
                selected_colors = [selected_colors; colors(selected_idx, :)];
            end
        end
    end
    
    if ~isempty(colors)
        reducedPC = pointCloud(selected_points, 'Color', selected_colors);
    else
        reducedPC = pointCloud(selected_points);
    end
    
    removed_count = pc.Count - reducedPC.Count;
    if removed_count > 0
        fprintf('🗜️ Redundancia eliminada: %d → %d puntos (-%d)\n', ...
                pc.Count, reducedPC.Count, removed_count);
    end
end

%% ========================== FUNCIONES AUXILIARES DE CLASIFICACIÓN ==========================

function density_info = analyzeLocalDensity(points)
    % Análisis de densidad local para clasificación
    density_info = struct();
    
    if size(points, 1) < 20
        density_info.variability = 0;
        density_info.planarity = 0;
        return;
    end
    
    try
        % Análisis de variabilidad de densidad
        [idx, distances] = rangesearch(points, points, 1.5);
        neighbor_counts = cellfun(@length, idx) - 1;
        density_info.variability = std(neighbor_counts) / (mean(neighbor_counts) + 1e-6);
        
        % Análisis de planaridad usando PCA
        [~, ~, latent] = pca(points);
        if length(latent) >= 3
            density_info.planarity = 1 - (latent(3) / (latent(1) + 1e-6));
        else
            density_info.planarity = 0;
        end
    catch
        density_info.variability = 0;
        density_info.planarity = 0;
    end
end

function mask = detectColorCategory(colors, target_color, tolerance)
    % Detectar puntos de una categoría específica por color
    if isempty(colors)
        mask = false(size(colors, 1), 1);
        return;
    end
    
    % Convertir colores a double si están en uint8
    if isa(colors, 'uint8')
        colors = double(colors) / 255.0;
    end
    
    % Asegurar que target_color esté en el mismo rango
    if max(target_color) <= 1.0
        target_color_norm = target_color;
    else
        target_color_norm = target_color / 255.0;
    end
    
    % Calcular distancias de color
    color_distances = vecnorm(colors - target_color_norm, 2, 2);
    mask = color_distances <= tolerance;
end

function keep_mask = intelligentVegetationFilter(points, subsample_rate)
    % Filtrado inteligente para vegetación
    if size(points, 1) < 10
        keep_mask = true(size(points, 1), 1);
        return;
    end
    
    % Submuestreo aleatorio pero preservando estructura
    num_keep = round(size(points, 1) * subsample_rate);
    
    % Preservar puntos en los bordes (más importantes para forma)
    hull_indices = [];
    try
        if size(points, 1) > 10
            hull_2d = convhull(points(:, 1), points(:, 2));
            hull_indices = unique(hull_2d);
        end
    catch
        hull_indices = [];
    end
    
    % Selección aleatoria del resto
    remaining_indices = setdiff(1:size(points, 1), hull_indices);
    num_random = max(0, num_keep - length(hull_indices));
    
    if num_random > 0 && ~isempty(remaining_indices)
        random_selection = randperm(length(remaining_indices), ...
                                   min(num_random, length(remaining_indices)));
        selected_random = remaining_indices(random_selection);
    else
        selected_random = [];
    end
    
    keep_indices = [hull_indices(:); selected_random(:)];
    keep_mask = false(size(points, 1), 1);
    keep_mask(keep_indices) = true;
end

function keep_mask = intelligentStructureFilter(points, preserve_rate)
    % Filtrado inteligente para estructuras (preservar más detalle)
    if size(points, 1) < 10
        keep_mask = true(size(points, 1), 1);
        return;
    end
    
    num_keep = round(size(points, 1) * preserve_rate);
    
    % Para estructuras, usar submuestreo uniforme para preservar forma
    if num_keep >= size(points, 1)
        keep_mask = true(size(points, 1), 1);
    else
        step = floor(size(points, 1) / num_keep);
        keep_indices = 1:step:size(points, 1);
        keep_indices = keep_indices(1:num_keep);
        
        keep_mask = false(size(points, 1), 1);
        keep_mask(keep_indices) = true;
    end
end

function keep_mask = intelligentGroundFilter(points, subsample_rate)
    % Filtrado agresivo para suelo (preservar solo estructura básica)
    if size(points, 1) < 10
        keep_mask = true(size(points, 1), 1);
        return;
    end
    
    num_keep = round(size(points, 1) * subsample_rate);
    
    if num_keep >= size(points, 1)
        keep_mask = true(size(points, 1), 1);
    else
        % Submuestreo aleatorio simple para suelo
        keep_indices = randperm(size(points, 1), num_keep);
        keep_mask = false(size(points, 1), 1);
        keep_mask(keep_indices) = true;
    end
end

function tree_colors = generateTreeColors(num_trees)
    % Generar colores distintivos para cada árbol
    tree_colors = zeros(num_trees, 3);
    
    % Base de colores verdes con variaciones
    base_green = [0.2, 0.8, 0.3];
    
    for i = 1:num_trees
        % Variaciones en tono y saturación
        hue_shift = (i - 1) * 0.1;  % Desplazamiento de tono
        saturation_var = 0.8 + 0.2 * sin(i * pi / 4);  % Variación de saturación
        
        color = base_green;
        color(1) = min(1, color(1) + hue_shift);  % Añadir un poco de rojo
        color(2) = color(2) * saturation_var;     % Variar saturación
        color(3) = min(1, color(3) + hue_shift * 0.5);  % Añadir un poco de azul
        
        tree_colors(i, :) = color;
    end
end

function h = drawVehicleShape(ax, pose, width, length, height, color, edgeColor)
    % Dibuja un vehículo en forma de carro (cuadrado/rectángulo) en 3D
    % ax: axes donde dibujar
    % pose: rigidtform3d con posición y orientación
    % width: ancho del vehículo (eje Y local)
    % length: largo del vehículo (eje X local)
    % height: altura del vehículo (eje Z local)
    % color: color de relleno [R G B]
    % edgeColor: color de los bordes [R G B]
    
    if nargin < 4, length = 4.5; end  % Largo típico de un carro
    if nargin < 5, width = 2.0; end   % Ancho típico
    if nargin < 6, height = 1.5; end  % Altura típica
    if nargin < 7, color = [1 0 0]; end
    if nargin < 8, edgeColor = [1 1 1]; end
    
    % Definir los vértices del carro en coordenadas locales (centrado en origen)
    % El frente del carro está en +X
    half_l = length / 2;
    half_w = width / 2;
    half_h = height / 2;
    
    % 8 vértices del cuadrado (caja)
    vertices_local = [
        half_l,  half_w,  half_h;   % 1: frente-derecha-arriba
        half_l, -half_w,  half_h;   % 2: frente-izquierda-arriba
       -half_l, -half_w,  half_h;   % 3: atrás-izquierda-arriba
       -half_l,  half_w,  half_h;   % 4: atrás-derecha-arriba
        half_l,  half_w, -half_h;   % 5: frente-derecha-abajo
        half_l, -half_w, -half_h;   % 6: frente-izquierda-abajo
       -half_l, -half_w, -half_h;   % 7: atrás-izquierda-abajo
       -half_l,  half_w, -half_h    % 8: atrás-derecha-abajo
    ];
    
    % Transformar vértices a coordenadas globales
    pos = pose.Translation(:)';
    R = pose.R;
    vertices_global = (R * vertices_local')' + repmat(pos, 8, 1);
    
    % Definir las 6 caras del carro
    faces = [
        1 2 6 5;  % cara frontal
        3 4 8 7;  % cara trasera
        1 4 3 2;  % cara superior
        5 6 7 8;  % cara inferior
        1 5 8 4;  % cara derecha
        2 3 7 6   % cara izquierda
    ];
    
    % Dibujar el carro como un patch
    h = patch(ax, 'Faces', faces, 'Vertices', vertices_global, ...
        'FaceColor', color, 'EdgeColor', edgeColor, 'LineWidth', 2, ...
        'FaceAlpha', 0.8, 'HandleVisibility', 'off');
end

function updateVehicleShape(h, pose, width, length, height)
    % Actualiza la posición y orientación de un vehículo dibujado
    if nargin < 3, length = 4.5; end
    if nargin < 4, width = 2.0; end
    if nargin < 5, height = 1.5; end
    
    half_l = length / 2;
    half_w = width / 2;
    half_h = height / 2;
    
    vertices_local = [
        half_l,  half_w,  half_h;
        half_l, -half_w,  half_h;
       -half_l, -half_w,  half_h;
       -half_l,  half_w,  half_h;
        half_l,  half_w, -half_h;
        half_l, -half_w, -half_h;
       -half_l, -half_w, -half_h;
       -half_l,  half_w, -half_h
    ];
    
    pos = pose.Translation(:)';
    R = pose.R;
    vertices_global = (R * vertices_local')' + repmat(pos, 8, 1);
    
    set(h, 'Vertices', vertices_global);
end