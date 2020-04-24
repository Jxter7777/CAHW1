#include "manifold.hpp"

#include <iostream>

Manifold::Manifold(
    std::shared_ptr<RigidBody2D> _body0, 
    std::shared_ptr<RigidBody2D> _body1,
    float2 _normal,
    float _penetration,
    bool _isHit)
    : m_body0(_body0), m_body1(_body1), m_normal(_normal),
      m_penetration(_penetration), m_isHit(_isHit)
    {}

void Manifold::Resolve() const
{
	// TODO
	if (m_isHit) {
		//非彈性碰撞
		float e = std::min(m_body0->m_restitution, m_body1->m_restitution);
		float2 Vel_diff = m_body1->m_velocity - m_body0->m_velocity; // 相對速度差
		float2 oth_normal = float2(-m_normal.y, m_normal.x); // 逆時針轉90度
		float Kf;
		float inner_product = Vel_diff.x*m_normal.x + Vel_diff.y*m_normal.y;// 正向力方向相對速度量 相撞的話<0
		float rel_oth_Vel = Vel_diff.x*oth_normal.x + Vel_diff.y*oth_normal.y;// 切線方向相對速度量 相擦的話<0
		/* 判斷是否有相對移動，選擇動摩擦靜摩擦 */
		if (rel_oth_Vel==0) 
			Kf = sqrt(m_body0->m_staticFriction * m_body1->m_staticFriction);
		else
			Kf = sqrt(m_body0->m_dynamicFriction * m_body1->m_dynamicFriction);

		float2 I = (1 + e)*m_normal*inner_product / (m_body0->m_invMass + m_body1->m_invMass); // 需要的動量改變量
		if(m_body0->m_invMass!=0)
			m_body0->AddVelocity(I*m_body0->m_invMass);
		if (m_body1->m_invMass != 0)
			m_body1->AddVelocity(-1*I*m_body1->m_invMass);
		/* 摩擦力計算 */
		float abs_I = sqrt(I.x*I.x + I.y*I.y);
		float Max_friction_force = abs_I * Kf * 1000.0f; // abs_I是1ms造成的動量改變，因此單位為力的話要乘以1000
		float force_need = 1.0f*1000.0f*rel_oth_Vel / (m_body0->m_invMass + m_body1->m_invMass);//完全非彈性
		float friction;
		
		if (Max_friction_force > abs(force_need)) {
			//printf("force_need = %f, Max_friction_force = %f, abs_I = %f, Kf = %f\n", force_need, Max_friction_force, abs_I, Kf);
			//printf("speed_x = %f, store_y=%f, speed_y = %f\n", m_body1->m_velocity.x, store_y, m_body1->m_velocity.y);
			friction = force_need;
		}
		else {//超過的話只能使用最大動摩擦or靜摩擦
			if(force_need>0)
				friction = Max_friction_force;
			else
				friction = -Max_friction_force;
		}
		I = friction * oth_normal;//增加方向
		if (m_body0->m_invMass != 0)
			m_body0->AddVelocity(I*m_body0->m_invMass * 0.001f);
		if (m_body1->m_invMass != 0)
			m_body1->AddVelocity(-I*m_body1->m_invMass * 0.001f);
	}
	// finished
}

void Manifold::PositionalCorrection() const
{
    const float percent = 0.4f; // usually 20% to 80%, when fps is 1/60
    const float slop = 0.01f;

	const float inv_mass_a = m_body0->GetInvMass();
	const float inv_mass_b = m_body1->GetInvMass();

    if(inv_mass_a == 0.0f && inv_mass_b == 0.0f)
        return;

    float2 correction = 
        (std::max( m_penetration - slop, 0.0f ) / (inv_mass_a + inv_mass_b))
        * percent * m_normal;

    m_body0->m_position -= inv_mass_a * correction;
    m_body1->m_position += inv_mass_b * correction;
}