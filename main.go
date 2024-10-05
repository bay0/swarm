// Package swarm provides an implementation of the Particle Swarm Optimization (PSO) algorithm.
// PSO is a computational method used to optimize a problem by iteratively improving a candidate solution
// with regard to a given measure of quality (fitness). It solves a problem by having a population (swarm)
// of candidate solutions (particles), moving these particles around in the search-space according to simple
// mathematical formulae over the particle's position and velocity.
package swarm

import (
	"fmt"
	"math"
	"math/rand"
	"sync"
	"time"
)

// Logger is a function type used for logging messages during the optimization process.
// Users can provide a custom logging function to handle log messages as desired.
type Logger func(string)

// Particle represents a single particle in the swarm.
// Each particle has a position and velocity in the search space,
// as well as its personal best known position and fitness value.
type Particle struct {
	Position     []float64 // Current position of the particle in the search space
	Velocity     []float64 // Current velocity of the particle
	BestPosition []float64 // Best known position of the particle
	BestFitness  float64   // Best known fitness value of the particle
}

// SwarmConfig contains configuration parameters for the swarm optimization algorithm.
// It defines the behavior and constraints of the swarm during the optimization process.
type SwarmConfig struct {
	Dimensions     int       // Number of dimensions in the search space
	SwarmSize      int       // Number of particles in the swarm
	MaxIterations  int       // Maximum number of iterations to run the optimization
	Inertia        float64   // Inertia weight influencing the particle's velocity
	CognitiveCoeff float64   // Cognitive coefficient (particle's own experience influence)
	SocialCoeff    float64   // Social coefficient (influence of the swarm's best-known position)
	MinPosition    []float64 // Minimum bounds for the particle positions
	MaxPosition    []float64 // Maximum bounds for the particle positions
	MinVelocity    []float64 // Minimum bounds for the particle velocities
	MaxVelocity    []float64 // Maximum bounds for the particle velocities
	LogFrequency   int       // Frequency of logging the swarm's best fitness (in iterations)
}

// Swarm represents the entire swarm of particles for the optimization process.
// It maintains the particles and the global best known position and fitness.
type Swarm struct {
	particles     []Particle              // Slice of particles in the swarm
	globalBest    []float64               // Best known position found by the swarm
	globalBestFit float64                 // Best known fitness value found by the swarm
	objectiveFunc func([]float64) float64 // Objective function to be minimized
	config        SwarmConfig             // Configuration parameters for the swarm
	logger        Logger                  // Logger function for logging messages
}

// NewSwarm creates a new Swarm with the given objective function, configuration, and logger.
// If no logger is provided, it defaults to using fmt.Println.
// The objective function should accept a slice of float64 representing a position in the search space
// and return a float64 representing the fitness value (to be minimized).
func NewSwarm(objFunc func([]float64) float64, config SwarmConfig, logger Logger) *Swarm {
	if logger == nil {
		logger = func(s string) { fmt.Println(s) }
	}
	return &Swarm{
		objectiveFunc: objFunc,
		config:        config,
		logger:        logger,
	}
}

// initialize sets up the initial state of the swarm by creating particles with random positions and velocities.
// It also initializes the global best position and fitness based on the initial particles.
func (s *Swarm) initialize() {
	s.particles = make([]Particle, s.config.SwarmSize)
	s.globalBestFit = math.Inf(1)

	for i := range s.particles {
		s.particles[i] = s.createParticle()
		if s.particles[i].BestFitness < s.globalBestFit {
			s.globalBestFit = s.particles[i].BestFitness
			s.globalBest = append([]float64{}, s.particles[i].Position...)
		}
	}
}

// createParticle initializes a new particle with random position and velocity within the specified bounds.
// It evaluates the particle's initial fitness and sets its personal best position and fitness.
func (s *Swarm) createParticle() Particle {
	position := make([]float64, s.config.Dimensions)
	velocity := make([]float64, s.config.Dimensions)
	for j := 0; j < s.config.Dimensions; j++ {
		position[j] = rand.Float64()*(s.config.MaxPosition[j]-s.config.MinPosition[j]) + s.config.MinPosition[j]
		velocity[j] = rand.Float64()*(s.config.MaxVelocity[j]-s.config.MinVelocity[j]) + s.config.MinVelocity[j]
	}
	bestPosition := append([]float64{}, position...)
	bestFitness := s.objectiveFunc(position)
	return Particle{Position: position, Velocity: velocity, BestPosition: bestPosition, BestFitness: bestFitness}
}

// updateParticle updates a single particle's position and velocity based on its own experience and the swarm's best known position.
// It applies the PSO velocity and position update equations and evaluates the particle's new fitness.
func (s *Swarm) updateParticle(p *Particle) {
	r1, r2 := rand.Float64(), rand.Float64()

	for d := 0; d < s.config.Dimensions; d++ {
		// Update velocity
		cognitive := s.config.CognitiveCoeff * r1 * (p.BestPosition[d] - p.Position[d])
		social := s.config.SocialCoeff * r2 * (s.globalBest[d] - p.Position[d])
		p.Velocity[d] = s.config.Inertia*p.Velocity[d] + cognitive + social

		// Clamp velocity within the specified bounds
		p.Velocity[d] = math.Max(s.config.MinVelocity[d], math.Min(p.Velocity[d], s.config.MaxVelocity[d]))

		// Update position
		p.Position[d] += p.Velocity[d]

		// Clamp position within the specified bounds
		p.Position[d] = math.Max(s.config.MinPosition[d], math.Min(p.Position[d], s.config.MaxPosition[d]))
	}

	// Evaluate fitness at the new position
	fitness := s.objectiveFunc(p.Position)

	// Update personal best if current fitness is better
	if fitness < p.BestFitness {
		p.BestFitness = fitness
		copy(p.BestPosition, p.Position)
	}
}

// Optimize runs the Particle Swarm Optimization algorithm.
// It iteratively updates the particles' positions and velocities to search for the optimal solution.
// The method returns the best position found by the swarm after the optimization process completes.
func (s *Swarm) Optimize() []float64 {
	s.initialize()
	start := time.Now()

	for iter := 0; iter < s.config.MaxIterations; iter++ {
		var wg sync.WaitGroup
		wg.Add(s.config.SwarmSize)

		for i := range s.particles {
			go func(i int) {
				defer wg.Done()
				s.updateParticle(&s.particles[i])
			}(i)
		}

		wg.Wait()

		// Update global best if any particle has a better fitness
		for _, p := range s.particles {
			if p.BestFitness < s.globalBestFit {
				s.globalBestFit = p.BestFitness
				copy(s.globalBest, p.BestPosition)
			}
		}

		// Log the current best fitness at specified intervals
		if s.config.LogFrequency > 0 && iter%s.config.LogFrequency == 0 {
			s.logger(fmt.Sprintf("Iteration %d: Best fitness = %f", iter, s.globalBestFit))
		}
	}

	elapsed := time.Since(start)
	s.logger(fmt.Sprintf("Optimization completed in %s", elapsed))
	s.logger(fmt.Sprintf("Best solution: %v", s.globalBest))
	s.logger(fmt.Sprintf("Best fitness: %f", s.globalBestFit))

	return s.globalBest
}
