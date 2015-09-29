#include <iostream>
#include <string>

#include "pxcsensemanager.h"
#include "pxcfaceconfiguration.h"

using namespace std;



class RobotTracking
{
public:
	//Constructor and destructor
	RobotTracking(){	}
	~RobotTracking(){	}
	void setup();
	void update();
	void cleanup();

private:
	PXCSenseManager *mSenseMgr;
	PXCFaceData *mFaceData;
};

void RobotTracking::setup()
{
	mSenseMgr = PXCSenseManager::CreateInstance();
	auto st = mSenseMgr->EnableFace();
	auto fm = mSenseMgr->QueryFace();
	auto cfg = fm->CreateActiveConfiguration();
	st = mSenseMgr->Init();

	cfg->pose.isEnabled = false;
	cfg->ApplyChanges();
	mFaceData = fm->CreateOutput();
}

void RobotTracking::update()
{
	if (mSenseMgr->AcquireFrame(true) >= PXC_STATUS_NO_ERROR)
	{
		if (!mFaceData)
			mFaceData = mSenseMgr->QueryFace()->CreateOutput();
		if (mFaceData)
		{
			mFaceData->Update();
			auto numFaces = mFaceData->QueryNumberOfDetectedFaces();
			//if (numFaces > 0)
			//	cout << "Found some faces" << endl;

			for (int i = 0; i < numFaces; ++i)
			{
				auto foundFace = mFaceData->QueryFaceByIndex(i);
				if (foundFace)
				{
					auto dd = foundFace->QueryDetection();
					if (dd)
					{
						PXCRectI32 outR;
						if (dd->QueryBoundingRect(&outR))
						{
							cout << "My X: " << outR.x << " My Y: " << outR.y << endl;
						}
					}
				}
			}
		}

		mSenseMgr->ReleaseFrame();
	}
}

void RobotTracking::cleanup()
{
	if (mFaceData)
		mFaceData->Release();
	mSenseMgr->Close();
}

int main()
{
	RobotTracking myRobotTracking;
	myRobotTracking.setup();

	while (true)
		myRobotTracking.update();
	myRobotTracking.cleanup();

}