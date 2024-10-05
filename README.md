# Swarm(PSO)

## Overview

This Go package provides an implementation of the Particle Swarm Optimization (PSO) algorithm. PSO is a computational method used to find optimal or near-optimal solutions to numerical and qualitative problems. It works by having a population (swarm) of candidate solutions (particles) move around in the search-space according to mathematical formulae over the particle's position and velocity.

## Features

- Flexible configuration of PSO parameters
- Concurrent update of particles for improved performance
- Customizable objective function
- Configurable logging
- Bounded search space and velocity

## Installation

To use this library in your Go project, you can install it using:

```bash
go get github.com/bay0/swarm
```

Replace `github.com/bay0/swarm` with the actual path where you've hosted this package.

## Usage

Here's a basic example of how to use the PSO library:

```go
package main

import (
    "fmt"
    "github.com/bay0/swarm"
)

func main() {
    // Define your objective function
    objectiveFunc := func(x []float64) float64 {
        return x[0]*x[0] + x[1]*x[1]  // Simple sphere function
    }

    // Configure the swarm
    config := swarm.SwarmConfig{
        Dimensions:     2,
        SwarmSize:      30,
        MaxIterations:  100,
        Inertia:        0.7,
        CognitiveCoeff: 1.4,
        SocialCoeff:    1.4,
        MinPosition:    []float64{-10, -10},
        MaxPosition:    []float64{10, 10},
        MinVelocity:    []float64{-1, -1},
        MaxVelocity:    []float64{1, 1},
        LogFrequency:   10,
    }

    // Create a new swarm
    s := swarm.NewSwarm(objectiveFunc, config, nil)

    // Run the optimization
    bestPosition := s.Optimize()

    fmt.Printf("Best solution found: %v\n", bestPosition)
    fmt.Printf("Objective value at best solution: %f\n", objectiveFunc(bestPosition))
}
```

## API Reference

### Types

#### `SwarmConfig`

Contains configuration parameters for the swarm optimization algorithm.

```go
type SwarmConfig struct {
    Dimensions     int
    SwarmSize      int
    MaxIterations  int
    Inertia        float64
    CognitiveCoeff float64
    SocialCoeff    float64
    MinPosition    []float64
    MaxPosition    []float64
    MinVelocity    []float64
    MaxVelocity    []float64
    LogFrequency   int
}
```

#### `Logger`

A function type used for logging messages during the optimization process.

```go
type Logger func(string)
```

### Functions

#### `NewSwarm`

Creates a new Swarm with the given objective function, configuration, and logger.

```go
func NewSwarm(objFunc func([]float64) float64, config SwarmConfig, logger Logger) *Swarm
```

### Methods

#### `Optimize`

Runs the Particle Swarm Optimization algorithm and returns the best position found.

```go
func (s *Swarm) Optimize() []float64
```

## Customization

### Custom Logger

You can provide a custom logging function when creating a new swarm. If no logger is provided, it defaults to using `fmt.Println`.

```go
customLogger := func(s string) {
    log.Printf("PSO: %s", s)
}
s := swarm.NewSwarm(objectiveFunc, config, customLogger)
```

### Objective Function

The objective function should accept a slice of `float64` representing a position in the search space and return a `float64` representing the fitness value (to be minimized).

```go
objectiveFunc := func(x []float64) float64 {
    // Your optimization problem goes here
    // Return the value to be minimized
}
```

## Performance Considerations

- The library uses goroutines to update particles concurrently, which can significantly improve performance for large swarms or complex objective functions.
- The performance of PSO heavily depends on the chosen parameters. Experiment with different values for `Inertia`, `CognitiveCoeff`, and `SocialCoeff` to find the best configuration for your specific problem.

## Contributing

Contributions to improve the library are welcome. Please feel free to submit issues or pull requests.

