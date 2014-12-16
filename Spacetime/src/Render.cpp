//======================================================================================================================//
// SpacetimeOptimization.cpp																							//
// (c) Ricky Arietta 2014																								//
// CIS 599 Masters Independent Study																					//
// University of Pennsylvania																							//
// Computer Graphics and Gaming Technology Program																		//
//																														//
// This code...																											//
//======================================================================================================================//

#include "Render.h"

Spacetime *render_system;
RenderUtil::Camera*	sCamera;

void motionCallback(int x, int y)
{
	sCamera->handleMotion(x, y);
}

void keyPress(const char key)
{
	switch(toupper(key)) {
		case 'P': render_system->switchPause(); break;
		default: break;
	}
}

void keyboardCallback(unsigned char key, int x, int y)
{
	if(key==27)
		exit(0);

	if(!sCamera->handleKey(key, x, y))
		keyPress(key);
}

void mouseCallback(int button, int state, int x, int y)
{
	sCamera->handleMouse(button, state, x, y);
}

void idleCallback()
{
	glutPostRedisplay();
}

void renderCallback()
{
	//render_system->Optimize();
	render_system->stepPhysics();

	RenderUtil::startRender(sCamera->getEye(), sCamera->getDir());
	
	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeSelectionFlag::eRIGID_DYNAMIC | PxActorTypeSelectionFlag::eRIGID_STATIC);
	if(nbActors)
	{
		std::vector<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeSelectionFlag::eRIGID_DYNAMIC | PxActorTypeSelectionFlag::eRIGID_STATIC, (PxActor**)&actors[0], nbActors);
		for (PxU32 i = 0; i < nbActors; i++) {
			PxVec3 color;
			if		(actors[i]->getName() == "base") color = PxVec3(1.0,0.0,0.0);
			else if (actors[i]->getName() == "leg1") color = PxVec3(0.0,1.0,0.0);
			else if (actors[i]->getName() == "leg2") color = PxVec3(0.0,0.0,1.0);
			else if (actors[i]->getName() == "bulb") color = PxVec3(0.0,1.0,1.0);
			else									 color = PxVec3(0.3,0.3,0.3);
			RenderUtil::renderActors(&actors[i], 1, true, color);
		}
	}

	const PxRenderBuffer& rb = scene->getRenderBuffer();
	for(PxU32 i = 0; i < rb.getNbLines(); i++)
	{
		const PxDebugLine& line = rb.getLines()[i];
		glLineWidth(2.5);
		float b = (PxU8)((line.color0>>16) & 0xff);
		float g = (PxU8)((line.color0>>8)  & 0xff);
		float r = (PxU8)((line.color0)     & 0xff);
		glColor3f(r, g, b);
		glBegin(GL_LINES);
		glVertex3f(line.pos0.x, line.pos0.y, line.pos0.z);
		glVertex3f(line.pos1.x, line.pos1.y, line.pos1.z);
		glEnd();
	}

	RenderUtil::finishRender();
}

void exitCallback(void)
{
	delete sCamera;
	render_system->cleanupPhysics();
}

void renderLoop(Spacetime *sys)
{
	render_system = sys;
	sCamera = new RenderUtil::Camera(PxVec3(35.0f, 35.0f, 35.0f), PxVec3(-35.0f,-35.0f,-35.0f));

	RenderUtil::setupDefaultWindow("Spacetime Optimization Example");
	RenderUtil::setupDefaultRenderState();

	glutIdleFunc(idleCallback);
	glutDisplayFunc(renderCallback);
	glutKeyboardFunc(keyboardCallback);
	glutMouseFunc(mouseCallback);
	glutMotionFunc(motionCallback);
	motionCallback(0,0);

	atexit(exitCallback);

	//render_system->initPhysics();
	glutMainLoop();
}
