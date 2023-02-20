\page generator Generator Configfile

This page provides definitions on how to configure a YAML file that can be parsed by the generator function `GenerateConfiguredTracker()`. It features all objects that can be created. To configure objects of a specific type, first, write the class name and then an array that considers the parameters required for the object. 

### Body
<h4>[Body](classm3t_1_1Body.html)</h4>
<pre>
Body:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
</pre>

### Renderer Geometry
<h4>[RendererGeometry](classm3t_1_1RendererGeometry.html)</h4>
<pre>
RendererGeometry:
  - name: "name"
    bodies: ["body_name_1", "body_name_2"]
</pre>

### Color Histograms
<h4>[ColorHistograms](classm3t_1_1ColorHistograms.html)</h4>
<pre>
ColorHistograms:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
</pre>

### Camera
<h4>[LoaderColorCamera](classm3t_1_1LoaderColorCamera.html)</h4>
<pre>
LoaderColorCamera:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
</pre>
<h4>[RealSenseColorCamera](classm3t_1_1RealSenseColorCamera.html)</h4>
<pre>
RealSenseColorCamera:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
</pre>
<h4>[AzureKinectColorCamera](classm3t_1_1AzureKinectColorCamera.html)</h4>
<pre>
AzureKinectColorCamera:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
</pre>
<h4>[LoaderDepthCamera](classm3t_1_1LoaderDepthCamera.html)</h4>
<pre>
LoaderDepthCamera:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
</pre>
<h4>[RealSenseDepthCamera](classm3t_1_1RealSenseDepthCamera.html)</h4>
<pre>
RealSenseDepthCamera:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
</pre>
<h4>[AzureKinectDepthCamera](classm3t_1_1AzureKinectDepthCamera.html)</h4>
<pre>
AzureKinectDepthCamera:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
</pre>

### Renderer
<h4>[FocusedDepthRenderer](classm3t_1_1FocusedDepthRenderer.html)</h4>
<pre>
FocusedDepthRenderer:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    renderer_geometry: "renderer_geometry_name"
    camera: "camera"
    referenced_bodies: ["body_name_1", "body_name_2"]
</pre>
<h4>[FocusedSilhouetteRenderer](classm3t_1_1FocusedSilhouetteRenderer.html)</h4>
<pre>
FocusedSilhouetteRenderer:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    renderer_geometry: "renderer_geometry_name"
    camera: "camera"
    referenced_bodies: ["body_name_1", "body_name_2"]
</pre>

### Model
<h4>[RegionModel](classm3t_1_1RegionModel.html)</h4>
<pre>
RegionModel:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
    body: "body_name"
    fixed_bodies: ["body_name_1", "body_name_2"]        #optional
    movable_bodies: ["body_name_1", "body_name_2"]      #optional
    fixed_same_region_bodies: ["body_name_1", "body_name_2"]    #optional
    movable_same_region_bodies: ["body_name_1", "body_name_2"]  #optional
</pre>
<h4>[DepthModel](classm3t_1_1DepthModel.html)</h4>
<pre>
DepthModel:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
    body: "body_name"
    occlusion_bodies: ["body_name_1", "body_name_2"]    #optional
</pre>

### Modality
<h4>[RegionModality](classm3t_1_1RegionModality.html)</h4>
<pre>
RegionModality:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    body: "body_name"
    color_camera: "color_camera_name"
    region_model: "region_model_name"
    measure_occlusions: {depth_camera: "depth_camera_name"}     #optional
    model_occlusions: {focused_depth_renderer: "focused_depth_renderer_name"}   #optional
    use_region_checking: {focused_silhouette_renderer: "focused_silhouette_renderer_name"}  #optional
    use_shared_color_histograms: {color_histograms: "color_histograms_name"}    #optional
</pre>
<h4>[DepthModality](classm3t_1_1DepthModality.html)</h4>
<pre>
DepthModality:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    body: "body_name"
    depth_camera: "depth_camera_name"
    depth_model: "depth_model_name"
    model_occlusions: {focused_depth_renderer: "focused_depth_renderer_name"}   #optional
    use_silhouette_checking: {focused_silhouette_renderer: "focused_silhouette_renderer_name"}  #optional
</pre>
<h4>[TextureModality](classm3t_1_1TextureModality.html)</h4>
<pre>
TextureModality:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    body: "body_name"
    color_camera: "color_camera_name"
    focused_silhouette_renderer: "focused_silhouette_renderer_name"
    measure_occlusions: {depth_camera: "depth_camera_name"}     #optional
    model_occlusions: {focused_depth_renderer: "focused_depth_renderer_name"}   #optional
</pre>

### Link
Note that links that should be referenced as child links need to be defined before the link that references them.
<h4>[Link](classm3t_1_1Link.html)</h4>
<pre>
Link:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    body: "body_name"                                   #optional
    modalities: ["modality_name_1", "modality_name_2"]    #optional
    child_links: ["child_link_name_1", "child_link_name_2"]   #optional
</pre>

### Constraint
<h4>[Constraint](classm3t_1_1Constraint.html)</h4>
<pre>
Constraint:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    link1: "link1_name"
    link2: "link2_name"
</pre>

### SoftConstraint
<h4>[SoftConstraint](classm3t_1_1SoftConstraint.html)</h4>
<pre>
SoftConstraint:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    link1: "link1_name"
    link2: "link2_name"
</pre>

### Optimizer
<h4>[Optimizer](classm3t_1_1Optimizer.html)</h4>
<pre>
Optimizer:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    root_link: "root_link_name"
    constraints: ["constraint_name_1", "constraint_name_2"]   #optional
    soft_constraints: ["soft_constraint_name_1", "soft_constraint_name_2"]  #optional
</pre>

### Viewer
<h4>[ImageColorViewer](classm3t_1_1ImageColorViewer.html)</h4>
<pre>
ImageColorViewer:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    color_camera: "color_camera_name"
</pre>
<h4>[NormalColorViewer](classm3t_1_1NormalColorViewer.html)</h4>
<pre>
NormalColorViewer:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    color_camera: "color_camera_name"
    renderer_geometry: "renderer_geometry_name"
</pre>
<h4>[ImageDepthViewer](classm3t_1_1ImageDepthViewer.html)</h4>
<pre>
ImageDepthViewer:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    depth_camera: "depth_camera_name"
</pre>
<h4>[NormalDepthViewer](classm3t_1_1NormalDepthViewer.html)</h4>
<pre>
NormalDepthViewer:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    depth_camera: "depth_camera_name"
    renderer_geometry: "renderer_geometry_name"
</pre>

### Detector
<h4>[StaticDetector](classm3t_1_1StaticDetector.html)</h4>
<pre>
StaticDetector:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
    optimizer: "optimizer_name"
</pre>
<h4>[ManualDetector](classm3t_1_1ManualDetector.html)</h4>
<pre>
ManualDetector:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"
    optimizer: "optimizer_name"
    color_camera: "color_camera_name"
</pre>

### Refiner
<h4>[Refiner](classm3t_1_1Refiner.html)</h4>
<pre>
Refiner:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    optimizers: ["optimizer_name_1", "optimizer_name_2"]
</pre>

### Tracker
<h4>[Tracker](classm3t_1_1Tracker.html)</h4>
<pre>
Tracker:
  - name: "name"
    metafile_path: "path/to/metafile.yaml"              #optional
    optimizers: ["optimizer_name_1", "optimizer_name_2"]
    detectors: ["detector_name_1", "detector_name_2"]   #optional
    publishers: ["publisher_name_1", "publisher_name_2"]    #optional
    subscribers: ["subscriber_name_1", "subscriber_name_2"]   #optional
    refiners: ["refiner_name_1", "refiner_name_2"]      #optional
    viewers: ["viewer_name_1", "viewer_name_2"]         #optional
</pre>
