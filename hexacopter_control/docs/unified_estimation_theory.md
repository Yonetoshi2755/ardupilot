# Unified System Identification and State Estimation

## The Key Insight

System identification and EKF design are not separate tasks - they form a **unified estimation problem** where:

1. **States** include both dynamic states (position, velocity, attitude) AND system parameters (inertia, coefficients)
2. **The filter simultaneously estimates** states and parameters online
3. **Parameter uncertainty** directly affects state uncertainty and vice versa

## Traditional Approach (Suboptimal)

```
System ID (offline) → Fixed Parameters → EKF (online) → State Estimates
```

Problems:
- Parameters assumed constant
- No uncertainty propagation from parameters to states
- Cannot adapt to changing dynamics (payload changes, damage, wear)

## Unified Approach (Optimal)

```
Augmented EKF: [States + Parameters] → Simultaneous Estimation
```

Benefits:
- Parameters evolve with states
- Uncertainty correctly propagated
- Adapts to system changes in real-time

## Mathematical Formulation

### Augmented State Vector

```
x_aug = [x_state; x_param]

where:
x_state = [quaternion; angular_velocity; position; velocity; gyro_bias]  (16 states)
x_param = [inertia_diagonal; thrust_coeff; torque_coeff; damping]       (8 parameters)

Total: 24-state augmented EKF
```

### Process Model

```
State dynamics:
ẋ_state = f(x_state, u, x_param)  ← Parameters affect dynamics!

Parameter dynamics:
ẋ_param = w_param                  ← Random walk or slow variation

Combined:
ẋ_aug = [f(x_state, u, x_param); w_param]
```

### Key Relationships

1. **Inertia affects angular dynamics**:
   ```
   ω̇ = I^(-1)[τ - ω × (I·ω)]
   ```
   Uncertainty in I propagates to uncertainty in ω̇

2. **Thrust coefficient affects acceleration**:
   ```
   a = R·(k_T·Σu²)/m + g
   ```
   Uncertainty in k_T affects position/velocity estimation

3. **Cross-correlations develop**:
   ```
   P_state,param ≠ 0
   ```
   State estimation errors correlate with parameter errors

## Implementation Strategy

### 1. Augmented EKF Structure

```cpp
class UnifiedEstimator {
    static const int STATE_DIM = 16;
    static const int PARAM_DIM = 8;
    static const int AUG_DIM = STATE_DIM + PARAM_DIM;
    
    float x_aug[AUG_DIM];      // Augmented state
    float P_aug[AUG_DIM][AUG_DIM];  // Augmented covariance
    
    // Process noise
    float Q_state[STATE_DIM];  // State process noise
    float Q_param[PARAM_DIM];  // Parameter drift (small)
};
```

### 2. Jacobian Structure

The augmented Jacobian F has special structure:

```
F = [∂f_state/∂x_state  ∂f_state/∂x_param]
    [       0                    I          ]
```

Where ∂f_state/∂x_param captures how parameter uncertainty affects state evolution.

### 3. Observability Enhancement

Not all parameters are observable from normal flight. Enhancement strategies:

1. **Excitation maneuvers**: Periodic small perturbations
2. **Information-theoretic control**: Commands that maximize parameter observability
3. **Multi-timescale estimation**: Fast states, slow parameters

## Practical Considerations

### When to Use Unified Estimation

✅ **Recommended when**:
- System parameters unknown or changing
- High-performance control required
- Payload changes during flight
- Component degradation monitoring needed

❌ **Not necessary when**:
- Parameters well-known and constant
- Computational resources limited
- Simple hovering/station-keeping tasks

### Computational Complexity

- Standard EKF: O(n²) with n=16
- Augmented EKF: O(n²) with n=24
- Increase: ~2.25x computation

Mitigation:
- Use sparse matrix techniques
- Separate fast/slow dynamics
- Selective parameter estimation

## Example: Hexacopter with Payload Pickup

```cpp
// Mission: Pick up unknown payload
class AdaptiveHexacopter {
    UnifiedEstimator estimator;
    
    void on_payload_pickup() {
        // Increase parameter process noise
        estimator.Q_param[MASS_IDX] *= 10;
        estimator.Q_param[INERTIA_IDX] *= 10;
        
        // System automatically adapts to new dynamics
        // No manual retuning required!
    }
    
    void fly_with_adaptation() {
        // Controller uses current parameter estimates
        Matrix3f I = estimator.get_inertia_estimate();
        float mass = estimator.get_mass_estimate();
        
        // Uncertainty-aware control
        Matrix3f I_cov = estimator.get_inertia_covariance();
        // Use robust control when uncertainty high
    }
};
```

## Research Directions

1. **Dual Control**: Control inputs that balance:
   - Tracking performance
   - Parameter identifiability

2. **Sparse Estimation**: Identify which parameters matter most

3. **Learning-Based Priors**: Use ML to predict parameter distributions

4. **Fault Detection**: Parameter jumps indicate failures

## Conclusion

The unified approach to system identification and state estimation represents the theoretically optimal solution. By treating parameters as slow-varying states, we:

- Correctly propagate all uncertainties
- Adapt to changing dynamics automatically  
- Achieve better overall estimation accuracy
- Enable truly adaptive control

This is not just an implementation detail - it's a fundamental shift in how we think about estimation in dynamic systems.