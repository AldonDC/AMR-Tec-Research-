# 🎨 Assets - Recursos Gráficos del Proyecto

Este directorio contiene los recursos gráficos utilizados en el README del proyecto.

## 📁 Archivos Disponibles

### 1. `matlab_cover.svg` 
**Portada principal del repositorio GitHub**

- **Dimensiones**: 1280 x 640 px (formato óptimo para GitHub)
- **Formato**: SVG (vectorial, escalable sin pérdida de calidad)
- **Uso**: Imagen de encabezado del README.md principal
- **Características**:
  - Diseño profesional con paleta de colores MATLAB oficial
  - Incluye iconos de tecnologías: RTK-GPS, LiDAR 3D, SLAM, NDT/ICP
  - Efectos visuales: gradientes, grid background, glow effects
  - Animación sutil de línea de escaneo

**Vista previa**: Abre directamente el archivo `.svg` en cualquier navegador moderno.

---

### 2. `github_cover.html`
**Generador interactivo de portada**

Página HTML que genera una versión animada e interactiva de la portada con efectos adicionales:

- **Cómo usar**:
  ```bash
  # Opción 1: Doble clic en el archivo
  # Se abrirá automáticamente en tu navegador por defecto
  
  # Opción 2: Desde navegador
  # Arrastra el archivo al navegador
  
  # Opción 3: Servidor local
  cd assets
  python -m http.server 8000
  # Abre http://localhost:8000/github_cover.html
  ```

- **Características adicionales**:
  - Animaciones suaves con CSS
  - Efectos de hover en iconos de tecnologías
  - Línea de escaneo animada continua
  - Degradados dinámicos
  - Grid pattern de fondo

- **Para crear un screenshot**:
  1. Abre `github_cover.html` en tu navegador
  2. Presiona `F11` para pantalla completa (opcional)
  3. Usa la herramienta de captura:
     - **Windows**: Windows + Shift + S
     - **Mac**: Cmd + Shift + 4
     - **Linux**: Captura de pantalla / Flameshot
  4. Guarda como `matlab_cover.png` (1280x640 px)

---

## 🎨 Paleta de Colores Oficial

### MATLAB Blue Gradient
```css
Primario:   #0076A8  /* MATLAB Blue */
Secundario: #004d6d  /* Dark Blue */
Terciario:  #003d5c  /* Darker Blue */
```

### MATLAB Orange (Logo)
```css
Principal:  #FF6B35  /* MATLAB Orange */
Acento:     #FF8C42  /* Light Orange */
```

### Complementarios
```css
Gold:       #FFD700  /* Subtítulos destacados */
White:      #FFFFFF  /* Texto principal */
White 90%:  rgba(255,255,255,0.9)  /* Texto secundario */
White 15%:  rgba(255,255,255,0.15) /* Backgrounds */
```

---

## 🖼️ Directrices de Uso

### Para el README.md
```markdown
<!-- Usar la versión SVG (ligera, escalable) -->
![MATLAB Cover](./assets/matlab_cover.svg)
```

### Para Redes Sociales / Presentaciones
```markdown
<!-- Si necesitas PNG, genera desde github_cover.html -->
![MATLAB Cover](./assets/matlab_cover.png)
```

---

## 📐 Especificaciones Técnicas

### SVG (`matlab_cover.svg`)
- **Peso**: ~8 KB (extremadamente ligero)
- **Resolución**: Vectorial (infinita)
- **Compatibilidad**: Todos los navegadores modernos, GitHub
- **Edición**: Inkscape, Adobe Illustrator, o cualquier editor SVG

### HTML (`github_cover.html`)
- **Peso**: ~6 KB
- **Tecnologías**: HTML5 + CSS3 (vanilla, sin dependencias)
- **Navegadores**: Chrome 90+, Firefox 88+, Safari 14+, Edge 90+
- **Responsive**: Diseñado para 1280x640 fijo (óptimo para GitHub)

---

## 🔧 Personalización

### Cambiar Título
Edita el archivo SVG o HTML en la sección del texto principal:

```xml
<!-- En SVG -->
<text x="640" y="250" ...>
  Tu Nuevo Título Aquí
</text>
```

```html
<!-- En HTML -->
<h1 class="title">Tu Nuevo Título Aquí</h1>
```

### Modificar Colores
Busca los gradientes definidos en `<defs>`:

```xml
<linearGradient id="bgGradient" ...>
  <stop offset="0%" style="stop-color:#TU_COLOR_1;..."/>
  <stop offset="100%" style="stop-color:#TU_COLOR_2;..."/>
</linearGradient>
```

### Añadir/Quitar Iconos
Busca la sección "Tech Stack Icons" y duplica/elimina los grupos `<g>`:

```xml
<g transform="translate(X, Y)">
  <!-- Tu nuevo icono aquí -->
</g>
```

---

## 📊 Métricas de Rendimiento

```
Tiempo de carga SVG:    <100ms
Tamaño SVG:             ~8 KB
Tamaño HTML:            ~6 KB
Tiempo render HTML:     ~200ms
Compatibilidad GitHub:  100% ✅
```

---

## 🚀 Próximas Mejoras Planeadas

- [ ] Versión dark/light mode toggle
- [ ] Generador automático con parámetros personalizables
- [ ] Exportador PNG de alta resolución desde CLI
- [ ] Plantillas alternativas (minimalista, tech, académico)
- [ ] Versión animada en GIF para social media

---

## 📝 Licencia

Estos assets gráficos están bajo la misma licencia académica del proyecto principal.

---

## 🤝 Contribuciones

Si mejoras estos diseños o creas versiones alternativas:
1. Mantén la paleta de colores consistente
2. Documenta los cambios en este README
3. Incluye archivos fuente editables (SVG/HTML)
4. Respeta las dimensiones 1280x640 px para GitHub

---

<div align="center">

**Última actualización**: Octubre 2025

</div>
