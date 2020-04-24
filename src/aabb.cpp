#include "aabb.hpp"

#include "manifold.hpp"
#include "circle.hpp"
#include "collision.hpp"

#include "GL/freeglut.h"

Manifold AABB::accept(std::shared_ptr<const ShapeVisitor<Manifold>> visitor) const
{
    return visitor->visitAABB(shared_from_this());
}

Manifold AABB::visitAABB(std::shared_ptr<const AABB> _shape) const
{
    // TODO
	float2 distance = _shape->m_body->GetPosition() - m_body->GetPosition();
	float2 combine_size = (m_extent+ _shape->m_extent) / 2.0f;
	float2 Vel_diff = _shape->m_body->GetVelocity() - m_body->GetVelocity();
	float2 overlap = combine_size - abs(distance);
	float2 Normal = float2(0.0f, 0.0f);
	float penetration = 0.0f;
	bool isHit = false;
	if (overlap.x >= 0.0f && overlap.y >= 0.0f) {
		//isHit = true;
		if (overlap.x < overlap.y) {
			penetration = overlap.x;
			if (distance.x*Vel_diff.x < 0) //face to face
				isHit = true;

			if (distance.x >= 0.0f)
				Normal = float2(1.0f, 0.0f);
			else
				Normal = float2(-1.0f, 0.0f);
		}
		else {
			penetration = overlap.y;
			if (distance.y*Vel_diff.y < 0) //face to face
				isHit = true;

			if (distance.y >= 0.0f)
				Normal = float2(0.0f, 1.0f);
			else
				Normal = float2(0.0f, -1.0f);
			
		}		
	}
	//printf("%f,%f %f %d\n", overlap.x, overlap.y, penetration, isHit);
	return Manifold(
		m_body,
		_shape->m_body,
		Normal,
		penetration,
		isHit
	);
    return Manifold(
        m_body,
        _shape->m_body,
		float2(0.0f, 0.0f),
        0.0f,
        false
    );
}

Manifold AABB::visitCircle(std::shared_ptr<const Circle> _shape) const
{
    auto manifold = CollisionHelper::GenerateManifold(
        shared_from_this(),
        _shape
    );

    return manifold;
}

void AABB::Render() const
{
    glPushMatrix();

    glTranslatef(m_body->GetPosition().x, m_body->GetPosition().y, 0);

    glBegin(GL_LINE_LOOP);
    {
        float2 half_extent = m_extent / 2.0f;

        glVertex2f(0 - half_extent[0], 0 - half_extent[1]);
        glVertex2f(0 - half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 - half_extent[1]);
    }
    glEnd();

    glBegin(GL_POINTS);
    {
        glPushMatrix();

        glVertex2f(0, 0);

        glPopMatrix();
    }
    glEnd();

    glPopMatrix();
}