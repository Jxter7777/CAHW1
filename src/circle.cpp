#include "circle.hpp"

#include "manifold.hpp"
#include "aabb.hpp"
#include "collision.hpp"

#include "GL/freeglut.h"

#include <cmath>

Manifold Circle::accept(std::shared_ptr<const ShapeVisitor<Manifold>> visitor) const
{
    return visitor->visitCircle(shared_from_this());
}

Manifold Circle::visitAABB(std::shared_ptr<const AABB> _shape) const
{
    auto manifold = CollisionHelper::GenerateManifold(
        _shape,
        shared_from_this()
    );

    return manifold;
}

Manifold Circle::visitCircle(std::shared_ptr<const Circle> _shape) const
{
    // TODO
	float2 distance = _shape->m_body->GetPosition() - m_body->GetPosition();
	float combine_size = m_radius + _shape->m_radius;
	float2 Vel_diff = _shape->m_body->GetVelocity() - m_body->GetVelocity();
	float co_size = sqrt(distance.x*distance.x + distance.y*distance.y);
	float overlap = combine_size - co_size;
	float2 Normal = float2(0.0f, 0.0f);
	float penetration = 0.0f;
	
	bool isHit = false;
	if (overlap>= 0.0f) {
		if (distance.x*Vel_diff.x + distance.y*Vel_diff.y < 0) //face to face
			isHit = true;
		penetration = overlap;
		Normal = distance / co_size;
	}
	//printf("%f,%f %f %d\n", overlap.x, overlap.y, penetration, isHit);
	return Manifold(
		m_body,
		_shape->m_body,
		Normal,
		penetration,
		isHit
	);
	
	//This is a template return object, you should remove it and return your own Manifold
    return Manifold(
        m_body,
        _shape->m_body,
        float2(0.0f, 0.0f),
        0.0f,
        false
    );
}

void Circle::Render() const
{
    const size_t k_segments = 200;

    glPushMatrix();
    glBegin(GL_LINE_LOOP);
    {
        float theta = 0.0f;
        float inc = (float)M_PI * 2.0f / k_segments;
        for(size_t i = 0; i < k_segments; ++i)
        {
            theta += inc;
            float2 p( std::cos( theta ), std::sin( theta ) );
            p *= m_radius;
            p += m_body->GetPosition();
            glVertex2f( p.x, p.y );
        }
    }
    glEnd( );
    glPopMatrix();
}