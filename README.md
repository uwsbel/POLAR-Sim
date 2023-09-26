# POLAR3D
a database of bounding box labels and terrain meshes for the <a href="https://ti.arc.nasa.gov/dataset/IRG_PolarDB/" target="_blank">POLAR dataset</a>

# Cover Photos
Pictures in the *CoverPhotos* folder show how we index the rocks in each terrain. The indices do not meet the label orders in the bounding box label txt files. The indices meet the rock ID of the mesh files in each terrain. Original pictures come from the POLAR dataset.

# Bounding Box Labels
Please check the *Labels* folder. The bounding box label files in YOLO format are categorized in the terrain ID folders. Each txt file meets with one HDR photo of the POLAR dataset.

- The label files are named as the following rule: [terrain ID]\_[stereo camera position]\_[rover light on/off]\_[Sun azimuth]\_[Left/Right camera of the stereo camera]\_[exposure time in millisecond], and "no" in [Sun azimuth] means that there was no simulated Sun (the spot light was turned off).

- The photos of very low exposure time may not have shadow or rock labels, since the photos are so dark that the rocks and shadows are judged invisible by the annotator.

- If the photos have no corresponding label files, it means that the photos have no labels (usually due to nothing visible).

# Terrain Meshes (the separated ground and rocks)
(Until now only Terrains 1, 4, and 11 finished. The rest terrains will be done and uploaded soon.)

Please check the *TerrainMeshes* folder. Meshes of the separated ground and rocks (rock IDs can be referred in the cover photos) of each terrain are built in obj files. 



# Questions And Bug Logs
Please raise Issues if you have any questions or find any bug. Thank you!

# Copyright
All rights reserved by the Simulation Based Engineering Lab in the University of Wisconsin-Madison
