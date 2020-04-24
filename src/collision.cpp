#include "collision.hpp"

#include <algorithm>
#include <iostream>

#include "linalg.h"

Manifold CollisionHelper::GenerateManifold(std::shared_ptr<const AABB> _a, std::shared_ptr<const Circle> _b)
{
	// TODO
	typedef linalg::aliases::float2 float2;
	float2 distance = _b->m_body->GetPosition() - _a->m_body->GetPosition();
	//float2 combine_size = float2(_a->m_extent.x+ _b->m_radius, _a->m_extent.y + _b->m_radius);
	float2 combine_size = _a->m_extent / 2.0f + _b->m_radius;
	float2 overlap = combine_size - abs(distance);
	float2 Normal = float2(0.0f, 0.0f);
	float penetration = 0.0f;
	bool isHit = false;
	bool isTouch = false;
	float2 corner = _a->m_body->GetPosition();
	if (distance.x > 0) 
		corner += float2(_a->m_extent.x / 2.0f, 0.0f);
	else
		corner -= float2(_a->m_extent.x / 2.0f, 0.0f);
	if (distance.y > 0)
		corner += float2(0.0f, _a->m_extent.y / 2.0f);
	else
		corner -= float2(0.0f, _a->m_extent.y / 2.0f);
	float2 Vel_diff = _b->m_body->GetVelocity() - _a->m_body->GetVelocity();
	float2 co_dis = _b->m_body->GetPosition() - corner;
	float co_dis_abs = sqrt(co_dis.x*co_dis.x + co_dis.y*co_dis.y);
	//if (overlap.x >= 0.0f && overlap.y >= 0.0f) {
	//}
	//if(overlap.x>=_b->m_radius||overlap.y>=_b->m_radius|| co_dis_abs<=)
	
	if (overlap.x >= 0.0f && overlap.y >= 0.0f) {
		//printf("%f, %f\n", overlap.x, overlap.y);
		if (overlap.x > _b->m_radius && overlap.y > _b->m_radius) {//in
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
		else {//out, find the closest
			if (overlap.x < _b->m_radius&&overlap.y < _b->m_radius) {
				if (co_dis_abs < _b->m_radius) {
					penetration = _b->m_radius - co_dis_abs;
					if(co_dis.x*Vel_diff.x + co_dis.y*Vel_diff.y <0)
						isHit = true;
					Normal = co_dis / co_dis_abs;
				}
			}
			else if (overlap.x < overlap.y) {
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
			;
		}
	}
	return Manifold(
		_a->m_body,
		_b->m_body,
		Normal,
		penetration,
		isHit
	);
	
	return Manifold(
		_a->m_body,
		_b->m_body,
		linalg::aliases::float2(0.0f, 0.0f),
        0.0f,
        false
    );
}