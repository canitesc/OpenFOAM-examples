# Smoke Plume Simulation - OpenFOAM Buoyancy-Driven Flow

## Overview

This OpenFOAM case simulates a buoyant smoke plume rising from a hot inlet positioned at the bottom of an enclosed rectangular domain. The simulation demonstrates fundamental buoyancy-driven flow phenomena that occur when hot gases rise through cooler ambient air, similar to the behavior observed in smoke rising from a fire, hot gas emissions from industrial stacks, or natural convection patterns in enclosed spaces. 

The computational setup employs OpenFOAM's `buoyantPimpleFoam` solver, which is specifically designed to handle compressible flow problems where buoyancy effects play a significant role in the fluid dynamics. This solver couples the momentum, energy, and continuity equations to accurately model the complex interaction between temperature gradients, density variations, gravity forces, and the resulting fluid motion. The smoke plume simulation implemented in the `smokePlume` directory draws inspiration from the [PhiFlow smoke plume example](https://tum-pbs.github.io/PhiFlow/examples/grids/Smoke_Plume.html), but has been fully reimplemented using OpenFOAM's advanced numerical framework and the buoyantPimpleFoam solver to leverage OpenFOAM's robust handling of compressible buoyant flows.

## Simulation Result

![Smoke Plume Simulation](https://github.com/canitesc/OpenFOAM-examples/raw/main/smokePlume/smoke.gif)

## Physical Problem Description

### Domain Geometry

The computational domain consists of a rectangular channel with dimensions of 1 meter in width, 2 meters in height, and 0.1 meters in depth, creating an effectively quasi-two-dimensional simulation environment. At the bottom boundary of this domain, a small inlet opening spans 0.2 meters in width, precisely centered between x-coordinates of 0.4m and 0.6m, through which the hot gas is injected into the domain. 

The domain is completely enclosed with solid wall boundaries on all sides except for the inlet opening, meaning there is no designated outlet for the flow to escape. This enclosed configuration forces the injected hot gas to recirculate within the domain, creating complex flow patterns as the buoyant plume interacts with the ceiling and walls. The computational mesh employed for spatial discretization consists of a structured hexahedral grid with 100 cells in the horizontal direction and 160 cells in the vertical direction, resulting in a total of 16,000 computational cells that provide sufficient resolution to capture the essential flow features while maintaining reasonable computational costs.

### Physics Modeled

The simulation captures several fundamental physical phenomena that govern buoyancy-driven flows. The primary driving mechanism is the injection of hot air at a temperature of 373 Kelvin (100°C) into a domain initially filled with cooler ambient air at 293 Kelvin (20°C). This significant temperature difference of 80 Kelvin creates substantial density variations within the fluid, as described by the ideal gas law.

Natural convection arises as the fundamental transport mechanism, where the temperature-induced density differences generate buoyancy forces that drive the upward motion of the heated fluid. The lighter hot gas experiences an upward buoyancy force when surrounded by denser cool air, causing it to rise through the domain in a characteristic plume structure. The simulation treats the flow as laminar, without turbulence modeling, which allows for cleaner visualization of the fundamental flow structures and reduces numerical diffusion that might otherwise obscure the plume dynamics.

The solver accounts for compressibility effects through the perfect gas equation of state, which relates pressure, density, and temperature according to the relationship ρ = p/(RT), where R is the specific gas constant for air. This approach allows the simulation to accurately capture the density variations that drive the buoyant motion. Gravity acts uniformly throughout the domain with a magnitude of 9.81 m/s² in the negative y-direction, providing the body force that, combined with density variations, generates the buoyancy effects central to this flow problem.

### Key Phenomena Observed

The simulation reveals several characteristic features of buoyancy-driven flows in enclosed domains. The most prominent feature is the formation and rise of the buoyant plume, which develops as the hot gas injected at the inlet accelerates upward due to the buoyancy forces arising from temperature-induced density differences. The plume maintains a relatively coherent structure as it rises through the cooler ambient fluid, entraining surrounding air and gradually expanding in width.

When the rising plume impinges on the upper boundary of the domain, it undergoes a dramatic transformation, forming a characteristic mushroom cloud pattern. This occurs because the vertical momentum of the plume is redirected horizontally upon collision with the ceiling, causing the hot gas to spread laterally in both directions. The interaction creates a distinctive cap-like structure reminiscent of the mushroom clouds observed in atmospheric phenomena or explosive events.

As the simulation progresses, the hot gas accumulates near the ceiling and begins to spread laterally throughout the upper portion of the domain. This lateral spreading, combined with the continuous injection of hot gas from below, establishes complex recirculation patterns within the enclosed space. Cool air descends along the side walls to replace the rising hot air, creating large-scale circulation cells that transport heat throughout the domain.

The rising plume also exhibits Rayleigh-Bénard type instabilities, which manifest as oscillations and asymmetries in the plume structure. These instabilities arise from the competition between the stabilizing effect of viscosity and the destabilizing effect of buoyancy, and their presence adds realistic complexity to the flow field. The exact nature and onset of these instabilities depend on the governing dimensionless parameters, particularly the Rayleigh number, which characterizes the relative importance of buoyancy and viscous forces in the system.

## File Structure

```
smokePlume/
├── 0/                    # Initial and boundary conditions
│   ├── T                 # Temperature field [K]
│   ├── U                 # Velocity field [m/s]
│   ├── p                 # Pressure field [Pa]
│   ├── p_rgh            # Pressure minus hydrostatic component [Pa]
│   ├── k                # Turbulent kinetic energy [m²/s²] (if using RAS)
│   ├── epsilon          # Turbulent dissipation rate [m²/s³] (if using RAS)
│   ├── alphat           # Turbulent thermal diffusivity [kg/m/s]
│   └── nut              # Turbulent kinematic viscosity [m²/s]
├── constant/            
│   ├── g                # Gravity vector (0 -9.81 0) [m/s²]
│   ├── thermophysicalProperties  # Fluid properties (air as perfect gas)
│   ├── turbulenceProperties      # Turbulence model settings
│   └── polyMesh/        # Mesh files (generated by blockMesh)
├── system/              
│   ├── controlDict      # Time control and output settings
│   ├── fvSchemes        # Numerical discretization schemes
│   ├── fvSolution       # Linear solver settings and pressure reference
│   └── blockMeshDict    # Mesh generation parameters
└── run.sh               # Convenience script to run simulation
```

## Key Parameters to Modify

### 1. Inlet Conditions (`0/U` and `0/T`)
```
# Velocity (0/U)
inlet {
    type            fixedValue;
    value           uniform (0 0.6 0);  # [x, y, z] m/s
}

# Temperature (0/T)
inlet {
    type            fixedValue;
    value           uniform 373;  # Kelvin
}
```
- Increase velocity for stronger plume
- Increase temperature for more buoyancy

### 2. Mesh Resolution (`system/blockMeshDict`)
```
blocks (
    hex (...) (40 160 1) simpleGrading (1 1 1)  # (nx ny nz)
    ...
);
```
- Increase numbers for finer mesh
- Keep nz=1 for 2D simulation

### 3. Turbulence Model (`constant/turbulenceProperties`)
```
simulationType  laminar;  # or RAS for turbulent flow
```
- `laminar`: Clean, smooth plume (good for visualization)
- `RAS` with `kEpsilon`: More realistic turbulent mixing

### 4. Time Control (`system/controlDict`)
```
endTime         10;          # Simulation duration [s]
deltaT          0.0005;      # Initial time step [s]
writeInterval   0.1;         # Output frequency [s]
maxCo           0.3;         # Maximum Courant number
```
- Reduce `maxCo` for more stable simulation
- Reduce `writeInterval` for more output frames

### 5. Numerical Schemes (`system/fvSchemes`)
```
divSchemes {
    div(phi,U)      Gauss linear;         # Central differencing (less diffusive)
    div(phi,h)      Gauss limitedLinear 1; # TVD scheme (more stable)
}
```
- `linear`: Less numerical diffusion, sharper plume
- `limitedLinear` or `linearUpwind`: More stable but diffusive

## Running the Simulation

### Basic Workflow
```bash
# 1. Clean previous results
rm -rf 0.* 1* 2* 3* 4* 5* 6* 7* 8* 9* 10 constant/polyMesh

# 2. Generate mesh
blockMesh

# 3. Check mesh quality
checkMesh

# 4. Run simulation
buoyantPimpleFoam

# 5. Visualize results
paraFoam
```

### Using the Run Script
```bash
./run.sh  # Does all the above automatically
```

## Visualization in ParaView

### Basic Temperature Visualization
1. Launch ParaView: `paraFoam &`
2. Click "Apply" to load the case
3. Select "T" (Temperature) from the dropdown menu
4. Adjust color scale (typically 293-373K)
5. Click play button to animate

### Available Fields for Visualization

#### Primary Fields
- **T (Temperature)** [K]: Shows hot plume rising (293-373K range)
- **U (Velocity)** [m/s]: Vector field showing flow patterns
- **p_rgh (Pressure)** [Pa]: Pressure distribution minus hydrostatic component
- **p (Pressure)** [Pa]: Total pressure field

#### Derived Fields (in ParaView)
- **Velocity Magnitude**: `|U|` - Speed regardless of direction
- **Vorticity**: `curl(U)` - Shows rotational flow structures
- **Density**: Can be calculated from temperature using ideal gas law

### Advanced Visualization Techniques

#### 1. Velocity Vectors
- Filters → Common → Glyph
- Vectors: U
- Glyph Type: Arrow
- Scale Mode: vector

#### 2. Streamlines
- Filters → Common → Stream Tracer
- Vectors: U
- Seed Type: Line or Point Cloud
- Shows flow pathlines

#### 3. Contour Lines (Isotherms)
- Filters → Common → Contour
- Contour By: T
- Add values: 300, 310, 320, 330, 340, 350, 360K
- Shows temperature stratification

#### 4. Volume Rendering
- Change representation to "Volume"
- Adjust opacity transfer function
- Good for 3D visualization of temperature field

#### 5. Animation Export
- File → Save Animation
- Choose format (AVI, PNG sequence)
- Set frame rate and resolution

### Color Maps
Recommended color maps for temperature:
- **Cool to Warm**: Blue (cold) to Red (hot)
- **Black-Body Radiation**: Physically realistic
- **Rainbow**: High contrast but less perceptually uniform

## Troubleshooting

### Common Issues and Solutions

1. **Plume drifts to one side**
   - Natural due to Rayleigh-Bénard instability
   - Check pressure reference cell location in `system/fvSolution`
   - Ensure mesh is symmetric

2. **Too much numerical diffusion**
   - Switch to central differencing schemes
   - Increase mesh resolution
   - Reduce time step (lower maxCo)

3. **Simulation crashes**
   - Reduce inlet velocity
   - Reduce time step
   - Check mesh quality with `checkMesh`
   - Use more stable numerical schemes

4. **Plume too turbulent/chaotic**
   - Switch from RAS to laminar
   - Reduce inlet velocity
   - Increase mesh resolution

## Physics Background

### Governing Equations

The `buoyantPimpleFoam` solver employed in this simulation solves the complete set of compressible Navier-Stokes equations with buoyancy effects. These coupled partial differential equations govern the conservation of mass, momentum, and energy in the fluid system.

#### Continuity Equation (Mass Conservation)
The continuity equation ensures mass conservation throughout the domain:

$$\frac{\partial \rho}{\partial t} + \nabla \cdot (\rho \mathbf{u}) = 0$$

where $\rho$ is the fluid density, $t$ is time, and $\mathbf{u}$ is the velocity vector.

#### Momentum Equation
The momentum equation incorporates both pressure gradients and buoyancy forces:

$$\frac{\partial (\rho \mathbf{u})}{\partial t} + \nabla \cdot (\rho \mathbf{u} \mathbf{u}) = -\nabla p + \nabla \cdot \boldsymbol{\tau} + \rho \mathbf{g}$$

where $p$ is pressure, $\boldsymbol{\tau}$ is the viscous stress tensor given by $\boldsymbol{\tau} = \mu[\nabla \mathbf{u} + (\nabla \mathbf{u})^T - \frac{2}{3}(\nabla \cdot \mathbf{u})\mathbf{I}]$, $\mu$ is the dynamic viscosity, and $\mathbf{g}$ is the gravitational acceleration vector.

#### Energy Equation
The energy equation governs the transport of thermal energy:

$$\frac{\partial (\rho h)}{\partial t} + \nabla \cdot (\rho \mathbf{u} h) = \frac{\partial p}{\partial t} + \nabla \cdot (\alpha_{eff} \nabla h) + \mathbf{u} \cdot \nabla p$$

where $h$ is the specific enthalpy, and $\alpha_{eff} = \alpha + \alpha_t$ is the effective thermal diffusivity combining molecular ($\alpha$) and turbulent ($\alpha_t$) contributions.

#### Equation of State
The system is closed with the perfect gas equation of state:

$$\rho = \frac{p}{RT}$$

where $R = 287$ J/(kg·K) is the specific gas constant for air, and $T$ is the absolute temperature.

### Key Dimensionless Numbers

The flow behavior is characterized by several important dimensionless parameters that govern the relative importance of different physical effects.

#### Rayleigh Number
The Rayleigh number represents the ratio of buoyancy forces to viscous forces and thermal diffusion:

$$Ra = \frac{g\beta \Delta T L^3}{\nu \alpha} = \frac{\text{Buoyancy forces}}{\text{Viscous forces} \times \text{Thermal diffusion}}$$

where $g$ is gravitational acceleration, $\beta$ is the thermal expansion coefficient, $\Delta T$ is the characteristic temperature difference, $L$ is the characteristic length scale, $\nu$ is the kinematic viscosity, and $\alpha$ is the thermal diffusivity. For this simulation, with $\Delta T = 80$ K and $L = 2$ m, the Rayleigh number is approximately $Ra \sim 10^{9}$, indicating strong buoyancy effects.

#### Prandtl Number
The Prandtl number characterizes the relative rates of momentum and thermal diffusion:

$$Pr = \frac{\nu}{\alpha} = \frac{\mu c_p}{k} \approx 0.7$$

For air at atmospheric conditions, the Prandtl number remains nearly constant at approximately 0.7, indicating that momentum and heat diffuse at similar rates.

#### Richardson Number
The Richardson number quantifies the relative importance of buoyancy to shear forces:

$$Ri = \frac{g\beta \Delta T L}{U^2}$$

where $U$ is the characteristic velocity. When $Ri > 1$, buoyancy dominates over inertial forces, leading to stratified flow patterns.

### Physical Parameters

The simulation uses the following thermophysical properties for air, evaluated at a mean temperature of approximately 333 K (60°C):

The density varies according to the perfect gas law as $\rho = p/(RT)$, where the specific gas constant for air is $R = 287$ J/(kg·K). At atmospheric pressure and the mean temperature, the density ranges from approximately 1.2 kg/m³ in the cool regions to 0.95 kg/m³ in the hot plume.

The dynamic viscosity remains approximately constant at $\mu = 1.8 \times 10^{-5}$ Pa·s throughout the temperature range of interest. The specific heat capacity at constant pressure is $c_p = 1007$ J/(kg·K), which determines the amount of energy required to change the temperature of the air. The thermal expansion coefficient, given by $\beta = 1/T$ for an ideal gas, varies from approximately $3.4 \times 10^{-3}$ K⁻¹ in the cool regions to $2.7 \times 10^{-3}$ K⁻¹ in the hot plume, driving the buoyancy effects that dominate the flow dynamics.

## References
- OpenFOAM User Guide: www.openfoam.com
- Buoyant Flows: Turner, J.S. "Buoyancy Effects in Fluids"
- CFD Theory: Versteeg & Malalasekera "An Introduction to Computational Fluid Dynamics"