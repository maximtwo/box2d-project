#ifndef ConstantVolumeJoint_h__
#define ConstantVolumeJoint_h__

#include <vector>
#include <iostream>

using namespace std;

class ConstantVolumeJoint : public Test {

public:

	float radius = 1.5f;
	int numberOfBodies = 20;

	std::vector<b2Body*> bodies;
	b2ConstantVolumeJoint* cvj;

	b2ConstantVolumeJoint* blob;
		 
	ConstantVolumeJoint() {

		m_world->SetGravity(b2Vec2(0, -30));

		blob = creatBlob(0, 10, radius);		

		b2BodyDef bd;
		b2FixtureDef fd;
		fd.density = 1.0f;
		fd.friction = 1.0f;

		bd.type = b2_staticBody;
		bd.position.Set(0, 0);

		b2Body* groundBody;
		b2PolygonShape ps;
		ps.SetAsBox(5000, 1);
		fd.shape = &ps;
		
		groundBody = m_world->CreateBody(&bd);
		groundBody->CreateFixture(&fd);

	}

	float speed = 0.0f;
	float speedIncrement = 1.0f;

	virtual void Step(Settings* settings) {
		Test::Step(settings);

		auto bodies = blob->GetBodies();
		
		for (int i = 0; i < numberOfBodies; i++) {

			b2Body* a = bodies[i];
			b2Body* b = bodies[(i + 1) % numberOfBodies];

			b2Vec2 d = b->GetPosition() - a->GetPosition();
			d.Normalize();
			d *= speed * a->GetMass();

			a->ApplyLinearImpulse(d, a->GetWorldCenter(), true);
		}

	}

	b2ConstantVolumeJoint* creatBlob(float x, float y, float radius) {

		float angleStep = (360.0f / numberOfBodies) * 0.0174532925f;

		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.fixedRotation = true;

		b2FixtureDef fd;
		b2CircleShape cs;
		fd.shape = &cs;
		fd.friction = 1.0f;
		fd.density = 1.0f;
		cs.m_radius = 0.1f;

		b2ConstantVolumeJointDef cvjd;

		for (int i = 0; i < numberOfBodies; i++) {

			bd.position.Set(x + radius * cos(i * angleStep), y + radius * sin(i * angleStep));
			b2Body* body = m_world->CreateBody(&bd);

			body->CreateFixture(&fd);

			cvjd.AddBody(body);
		}

		return static_cast<b2ConstantVolumeJoint*>(m_world->CreateJoint(&cvjd));
	}

	b2Vec2 getBlobCenter(b2ConstantVolumeJoint* blob) {

		b2Vec2 center(0, 0);

		auto bodies = blob->GetBodies();

		for (unsigned i = 0; i < bodies.size(); ++i) {

			center += bodies[i]->GetPosition();
		}

		center.x /= bodies.size();
		center.y /= bodies.size();

		return center;
	}

	virtual void Keyboard(int key) {

		const int a = 65;
		const int d = 68;

		switch (key) {
			
		case a:
			speed += speedIncrement;
			break;
		case d:
			speed -= speedIncrement;
			break;			
		}
	}

	virtual void KeyboardUp(int key) {

		const int a = 65;
		const int d = 68;

		switch (key) {

		case a:
			speed -= speedIncrement;
			break;
		case d:
			speed += speedIncrement;
			break;
		}
		
	}

	virtual void MouseDown(const b2Vec2& p) {

		blob->Inflate(1.1f);
	}

	static Test* Create() {

		return new ConstantVolumeJoint();
	}
};

#endif // ConstantVolumeJoint_h__
