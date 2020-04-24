#include "integrator.hpp"

#include <algorithm>

#include "scene.hpp"

void ExplicitEulerIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
	// TODO
	const float2 gravity(0.0f, -9.8f);
	for (size_t i = 0; i < _bodies.size(); i++)
	{
		if (_bodies[i]->GetMass() != 0) {
			_bodies[i]->AddPosition(_bodies[i]->GetVelocity()*deltaTime);
			_bodies[i]->AddVelocity(_bodies[i]->GetForce()*deltaTime*_bodies[i]->GetInvMass());
			_bodies[i]->SetForce(_bodies[i]->GetMass()*gravity);
		}
		else {
			_bodies[i]->SetVelocity(float2(0.0f, 0.0f));// Á×§K¼vÅTÀR¤îª«Åé¸I¼²­pºâ
		}
		
	}
}

void RungeKuttaFourthIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
    if(scene == nullptr)
    {
        throw std::runtime_error("RungeKuttaFourthIntegrator has no target scene.");
    }

	const float2 gravity(0.0f, -9.8f);
	// TODO
	

	
}