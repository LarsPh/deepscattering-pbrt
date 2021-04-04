Deepscattering-PBRT
===============

A deeplearning accelerated cloud rendering extension based on pbrt-v3 renderer.

PBRT-v3 Renderer
--------------
See the [github page](https://github.com/mmp/pbrt-v3) for more details: 

Deepscattering Algorithm
--------------------
Base on the paper: [Deep Scattering: Rendering Atmospheric Clouds with Radiance-Predicting Neural Networks](https://la.disneyresearch.com/publication/deep-scattering/) and [Faster RPNN: Rendering Clouds with Latent Space Light Probes](https://dl.acm.org/doi/10.1145/3355088.3365150)

Finished Parts of the Project
--------------------
- A volume path tracer with tabulated chopped Mie phase function based on paper [Real-time realistic illumination and shading of stratiform clouds](http://www-evasion.imag.fr/Publications/2006/BNL06/)
- A adaptive sampler mentioned in the paper based confidence interval theory from paper [A statistical method for adaptive stochastic sampling](https://www.sciencedirect.com/science/article/abs/pii/009784938790029X)
- A random camera that generates rays as decribed in paper
- Sampling medium density and storing data in lmdb format for training
- Python code for training described in paper based on pytorch
- More than 70 vdb cloud data of different shape and textures
- Generating a small scale of data and expetiment training with them

Unfinished parts of the project
--------------------
- Generating the whole dataset and training

Current Results
--------------------
Comparison on hg and Mie phase function   
side view  
<img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/irre_7_s_mie.png?raw=true" width=50% height=50%><img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/irre_7_s_hg.png?raw=true" width=50% height=50%>  
back view  
<img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/irre_7_b_miefst.png?raw=true" width=50% height=50%><img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/irre_7_b_hg.png?raw=true" width=50% height=50%>  
front view  
<img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/irre_7_f_miefst.jpg?raw=true" width=50% height=50%><img src="https://github.com/LarsPh/deepscattering-pbrt/blob/data_generation/Gallery/irre_7_f_hg.jpg?raw=true" width=50% height=50%>

Training result on a small dataset

Problem faced and possible solutions
--------------------
- Currently as a hobbyist, it seems hard to find affordable hardware, i.e. cloud multi-cores cpu or gpu, thus data generating and training become time-consuming and expensive.
- A workaround could be improving efficiency of renderer: 
  - The power of GPU: [pbrt-v4](https://github.com/mmp/pbrt-v4) with optix (or just use optix)
  - Better algorithm: guided volume path tracing decribed in paper [Volume Path Guiding Based on Zero-Variance Random Walk Theory](https://dl.acm.org/doi/10.1145/3230635)

How to use
-------------
### building pbrt
  see original github page: https://github.com/mmp/pbrt-v3 
### choose different types of phase function
### Generating data
