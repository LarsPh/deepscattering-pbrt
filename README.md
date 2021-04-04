Deepscattering-PBRT
===============

A deep learning accelerated cloud rendering extension based on pbrt-v3 renderer.

PBRT-v3 Renderer
--------------
See the [github page](https://github.com/mmp/pbrt-v3) for more details: 

Deepscattering Algorithm
--------------------
Based on the paper: [Deep Scattering: Rendering Atmospheric Clouds with Radiance-Predicting Neural Networks](https://la.disneyresearch.com/publication/deep-scattering/) and [Faster RPNN: Rendering Clouds with Latent Space Light Probes](https://dl.acm.org/doi/10.1145/3355088.3365150)

Finished Parts of the Project
--------------------
- A volume path tracer with tabulated chopped Mie phase function based on paper [Real-time realistic illumination and shading of stratiform clouds](http://www-evasion.imag.fr/Publications/2006/BNL06/)
- A adaptive sampler mentioned in the paper based confidence interval theory from paper [A statistical method for adaptive stochastic sampling](https://www.sciencedirect.com/science/article/abs/pii/009784938790029X)
- A random camera that generates rays as described in paper
- Sampling medium density and storing data in lmdb format for training
- Python code for training described in paper based on pytorch
- More than 70 vdb cloud data of different shape and textures
- Generating a small scale of data and experiment training with them

Unfinished parts of the project
--------------------
- Generating the whole dataset and training

Current Results
--------------------
Comparison on hg and Mie phase function:  

side view  

<img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/irre_7_s_mie.png?raw=true" width=50% height=50%><img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/irre_7_s_hg.png?raw=true" width=50% height=50%>

back view

<img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/irre_7_b_miefst.png?raw=true" width=50% height=50%><img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/irre_7_b_hg.png?raw=true" width=50% height=50%>

front view

<img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/irre_7_f_miefst.jpg?raw=true" width=50% height=50%><img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/irre_7_f_hg.jpg?raw=true" width=50% height=50%>

A bonus: glory effect

<img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/glory.png?raw=true" width=50% height=50%><img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/glory_scaled.png?raw=true" width="120" height="120">

Problem faced and possible solutions
--------------------
- Currently as a hobbyist, it seems hard to find affordable hardware, i.e. cloud multi-cores cpu or gpu, thus data generating and training become time-consuming and expensive.
- A workaround could be improving efficiency of renderer: 
  - The power of GPU: [pbrt-v4](https://github.com/mmp/pbrt-v4) with optix (or just use optix)
  - Better algorithm: guided volume path tracing described in paper [Volume Path Guiding Based on Zero-Variance Random Walk Theory](https://dl.acm.org/doi/10.1145/3230635)

How to use
-------------
### building pbrt
  See original github page: https://github.com/mmp/pbrt-v3 
### choose between different types of phase function
  A new parameter "string phase_type" can be specified under participating media option. "hg" for hg phase function, "mie" for chopped mie phase function. And "mie_fst" for applying unchopped version only on the first bounce for a better silver light effect.
### Generating data
  Set integrator option to "volpath" which uses the modified integrator for data generation. Set camera and light option to "ds" and "randomdistant" which uses sampling approach described in section 5.1 of the paper. Use parameter "string dbpath" under film option to specify the file path for the data generated.
  
  Here's an example for the scene-wide options and light source parts of the input file.

~~~
Film "image" "integer xresolution" [ 96 ] "integer yresolution" [ 96 ] "string dbpath" [ "D:/db/large_61" ] // This will take 10000 samples
PixelFilter "gaussian" "float xwidth" [ 2 ] "float ywidth" [ 2 ]
Sampler "maxmindist" "integer pixelsamples" [ 1024 ]
Integrator "volpath" "integer maxdepth" [ 5000 ]
Accelerator "bvh"
Camera "ds"
	    
AttributeBegin
		LightSource "randomdistant" "rgb L" [2 2 2]
AttributeEnd
~~~

### Training the network
  Run the python script "/deep_part/deep_scattering_cpu.py". Parameter "mapSize" stands for the size of data file. Use "convert_to_text.py" to visualize data files.
