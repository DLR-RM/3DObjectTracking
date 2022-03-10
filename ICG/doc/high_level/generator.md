\page generator Generator Configfile

This page provides definitions on how to configure a YAML file that can be parsed by the generator function `GenerateConfiguredTracker()`. It features all objects that can be created. To configure objects of a specific type, first, write the class name and then an array that considers the parameters required for the object.

### Body
<h4>[Body](classicg_1_1Body.html)</h4>
```
Body:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
```

### Renderer Geometry
<h4>[RendererGeometry](classicg_1_1RendererGeometry.html)</h4>
```
RendererGeometry:
  - name: "name"
    bodies: ["body_name_1", "body_name_2"]
```

### Camera
<h4>[LoaderColorCamera](classicg_1_1LoaderColorCamera.html)</h4>
```
LoaderColorCamera:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
```
<h4>[RealSenseColorCamera](classicg_1_1RealSenseColorCamera.html)</h4>
```
RealSenseColorCamera:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
```
<h4>[AzureKinectColorCamera](classicg_1_1AzureKinectColorCamera.html)</h4>
```
AzureKinectColorCamera:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
```
<h4>[LoaderDepthCamera](classicg_1_1LoaderDepthCamera.html)</h4>
```
LoaderDepthCamera:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
```
<h4>[RealSenseDepthCamera](classicg_1_1RealSenseDepthCamera.html)</h4>
```
RealSenseDepthCamera:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
```
<h4>[AzureKinectDepthCamera](classicg_1_1AzureKinectDepthCamera.html)</h4>
```
AzureKinectDepthCamera:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
```

### Renderer
<h4>[FocusedDepthRenderer](classicg_1_1FocusedDepthRenderer.html)</h4>
```
FocusedDepthRenderer:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    renderer_geometry: "renderer_geometry_name"
    camera: "camera"
    referenced_bodies: ["body_name_1", "body_name_2"]
```

### Model
<h4>[RegionModel](classicg_1_1RegionModel.html)</h4>
```
RegionModel:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
    body: "body_name"
```
<h4>[DepthModel](classicg_1_1DepthModel.html)</h4>
```
DepthModel:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
    body: "body_name"
```

### Modality
<h4>[RegionModality](classicg_1_1RegionModality.html)</h4>
```
RegionModality:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    body: "body_name"
    color_camera: "color_camera_name"
    region_model: "region_model_name"
    measure_occlusions: {depth_camera: "depth_camera_name"}     #optional
    model_occlusions: {focused_depth_renderer: "focused_depth_renderer_name"}   #optional
```
<h4>[DepthModality](classicg_1_1DepthModality.html)</h4>
```
DepthModality:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    body: "body_name"
    depth_camera: "depth_camera_name"
    depth_model: "depth_model_name"
    model_occlusions: {focused_depth_renderer: "focused_depth_renderer_name"}   #optional
```

### Optimizer
<h4>[Optimizer](classicg_1_1Optimizer.html)</h4>
```
Optimizer:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    modalities: ["modality_name_1", "modality_name_2"]
```

### Viewer
<h4>[ImageColorViewer](classicg_1_1ImageColorViewer.html)</h4>
```
ImageColorViewer:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    color_camera: "color_camera_name"
```
<h4>[NormalColorViewer](classicg_1_1NormalColorViewer.html)</h4>
```
NormalColorViewer:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    color_camera: "color_camera_name"
    renderer_geometry: "renderer_geometry_name"
```
<h4>[ImageDepthViewer](classicg_1_1ImageDepthViewer.html)</h4>
```
ImageDepthViewer:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    depth_camera: "depth_camera_name"
```
<h4>[NormalDepthViewer](classicg_1_1NormalDepthViewer.html)</h4>
```
NormalDepthViewer:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    depth_camera: "depth_camera_name"
    renderer_geometry: "renderer_geometry_name"
```

### Detector
<h4>[StaticDetector](classicg_1_1StaticDetector.html)</h4>
```
StaticDetector:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
    body: "body_name"
```
<h4>[ManualDetector](classicg_1_1ManualDetector.html)</h4>
```
ManualDetector:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
    body: "body_name"
    color_camera: "color_camera_name"
```

### Refiner
<h4>[Refiner](classicg_1_1Refiner.html)</h4>
```
Refiner:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    optimizers: ["optimizer_name_1", "optimizer_name_2"]
```

### Tracker
<h4>[Tracker](classicg_1_1Tracker.html)</h4>
```
Tracker:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    optimizers: ["optimizer_name_1", "optimizer_name_2"]
    detectors: ["detector_name_1", "detector_name_2"]   #optional
    refiners: ["refiner_name_1", "refiner_name_2"]      #optional
    viewers: ["viewer_name_1", "viewer_name_2"]         #optional
```
