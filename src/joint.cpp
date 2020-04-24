#include "joint.hpp"

#include "GL/freeglut.h"

#include "rigidbody2D.hpp"
#include "util.hpp"

#include <iostream>

void SpringJoint::ApplyConstriant() const
{
    // TODO
	float2 distance = m_body0->GetPosition() - m_body1->GetPosition();
	float2 Vel_diff = m_body0->GetVelocity() - m_body1->GetVelocity();
	float inner_product = distance.x * Vel_diff.x + distance.y * Vel_diff.y;
	float dis_abs = sqrt(distance.x*distance.x + distance.y*distance.y);
	float2 f = -m_stiffness * (dis_abs - m_restLength)*(distance/dis_abs)- m_stiffness*(inner_product/dis_abs) * (distance / dis_abs);// 套公式
	if (m_body0->GetInvMass() != 0)
		m_body0->AddForce(f);
	if (m_body1->GetInvMass() != 0)
		m_body1->AddForce(-f);
}

void SpringJoint::Render() const
{
    glPushMatrix();
    glPushAttrib(GL_CURRENT_BIT);
    {
        glBegin(GL_LINES);
        // red for spring joint
        glColor3f(1, 0, 0);
        glVertex2f( m_body0->GetPosition().x, m_body0->GetPosition().y );
        glVertex2f( m_body1->GetPosition().x, m_body1->GetPosition().y );
        glEnd();
    }
    glPopAttrib();
    glPopMatrix();
}

void DistanceJoint::ApplyConstriant() const
{
	// TODO
	float2 distance = m_body0->GetPosition() - m_body1->GetPosition();
	float2 Vel_diff = m_body0->GetVelocity() - m_body1->GetVelocity();
	float inner_product = distance.x * Vel_diff.x + distance.y * Vel_diff.y;
	float dis_abs = sqrt(distance.x*distance.x + distance.y*distance.y);
	float2 relVel = inner_product * (distance / (dis_abs* dis_abs)); // 正向力方向相對速度
	float2 relDist = distance * (dis_abs - m_restLength) / dis_abs;
	float2 remove = relVel + relDist / m_deltaTime; // deltatime時間內回復原狀所需速度
	float2 f = remove / (m_body0->GetInvMass() + m_body1->GetInvMass());

	if (dis_abs >= m_restLength) { //超過長度才套用
		if (m_body0->GetInvMass()!=0)
			m_body0->AddVelocity(-f * m_body0->GetInvMass());
		if (m_body1->GetInvMass()!=0)
			m_body1->AddVelocity(f * m_body1->GetInvMass());
	}
}

void DistanceJoint::Render() const
{
    glPushMatrix();
    glPushAttrib(GL_CURRENT_BIT);
    {
        glBegin(GL_LINES);
        // green for distance joint
        glColor3f(0, 1, 0);
        glVertex2f( m_body0->GetPosition().x, m_body0->GetPosition().y );
        glVertex2f( m_body1->GetPosition().x, m_body1->GetPosition().y );
        glEnd();
    }
    glPopAttrib();
    glPopMatrix();
}