#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/common/common.hh>

#include <gazebo_grasp_plugin/GazeboGraspFix.h>
#include <gazebo_version_helpers/GazeboVersionHelpers.h>

#include <msgs/grasp_event.pb.h>

using gazebo::GazeboGraspFix;
using gazebo::GzVector3;

#define DEFAULT_GRIP_EFFORT_THRESHOLD 1.0
#define DEFAULT_ATTACH_DELAY 1.0
#define DEFAULT_DETACH_DELAY 0.1
#define DEFAULT_UPDATE_RATE 5
#define DEFAULT_MAX_GRIP_COUNT 10
#define DEFAULT_DISABLE_COLLISIONS_ON_ATTACH false

// Plugin elements have no SDF description, so GetElement() on a missing child
// makes sdformat print "Missing element description". Check for it first.
static sdf::ElementPtr GetOptionalElement(const sdf::ElementPtr &_sdf,
                                          const std::string &_name)
{
  if (!_sdf->HasElement(_name)) return sdf::ElementPtr();
  return _sdf->GetElement(_name);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboGraspFix)

////////////////////////////////////////////////////////////////////////////////
GazeboGraspFix::GazeboGraspFix()
{
  InitValues();
}

////////////////////////////////////////////////////////////////////////////////
GazeboGraspFix::GazeboGraspFix(physics::ModelPtr _model)
{
  InitValues();
}

////////////////////////////////////////////////////////////////////////////////
GazeboGraspFix::~GazeboGraspFix()
{
  // Release filter to make it safe to reload the model with plugin
  if (!filter_name.empty() && this->world)
  {
    physics::PhysicsEnginePtr physics = GetPhysics(this->world);
    physics::ContactManager *contactManager = physics->GetContactManager();
    if (contactManager)
        contactManager->RemoveFilter(filter_name);
  }
  this->update_connection.reset();
  if (this->node) this->node->Fini();
  this->node.reset();
}

////////////////////////////////////////////////////////////////////////////////
void GazeboGraspFix::Init()
{
  this->prevUpdateTime = common::Time::GetWallTime();
}

////////////////////////////////////////////////////////////////////////////////
void GazeboGraspFix::InitValues()
{
#if GAZEBO_MAJOR_VERSION > 2
  gazebo::common::Console::SetQuiet(false);
#endif

  // float timeDiff=0.25;
  // this->updateRate = common::Time(0, common::Time::SecToNano(timeDiff));
  this->prevUpdateTime = common::Time::GetWallTime();
  //float graspedSecs=2;
  //this->maxGripCount=floor(graspedSecs/timeDiff);
  //this->gripCountThreshold=floor(this->maxGripCount/2);
  this->node = transport::NodePtr(new transport::Node());
}

////////////////////////////////////////////////////////////////////////////////
void GazeboGraspFix::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  gzmsg << "Loading grasp-fix plugin" << std::endl;

  // ++++++++++++ Read parameters and initialize fields  +++++++++++++++

  physics::ModelPtr model = _parent;
  this->world = model->GetWorld();

  sdf::ElementPtr disableCollisionsOnAttachElem =
    GetOptionalElement(_sdf, "disable_collisions_on_attach");
  if (!disableCollisionsOnAttachElem)
  {
    gzmsg << "GazeboGraspFix: Using default " <<
          DEFAULT_DISABLE_COLLISIONS_ON_ATTACH <<
          " because no <disable_collisions_on_attach> element specified." <<
          std::endl;
    this->disableCollisionsOnAttach = DEFAULT_DISABLE_COLLISIONS_ON_ATTACH;
  }
  else
  {
    std::string str = disableCollisionsOnAttachElem->Get<std::string>();
    bool bVal = false;
    if ((str == "true") || (str == "1"))  bVal = true;
    this->disableCollisionsOnAttach = bVal;
    gzmsg << "GazeboGraspFix: Using disable_collisions_on_attach " <<
          this->disableCollisionsOnAttach << std::endl;
  }

  if (_sdf->HasElement("forces_angle_tolerance"))
  {
    gzwarn << "GazeboGraspFix: <forces_angle_tolerance> is obsolete and ignored; " <<
              "MobiPick grasps require contact on both finger sides." << std::endl;
  }

  sdf::ElementPtr gripEffortThresholdElem =
    GetOptionalElement(_sdf, "grip_effort_threshold");
  if (!gripEffortThresholdElem)
  {
    this->gripEffortThreshold = DEFAULT_GRIP_EFFORT_THRESHOLD;
    gzmsg << "GazeboGraspFix: Using default grip_effort_threshold " <<
          this->gripEffortThreshold << " Nm." << std::endl;
  }
  else
  {
    this->gripEffortThreshold = gripEffortThresholdElem->Get<double>();
    gzmsg << "GazeboGraspFix: Using grip_effort_threshold " <<
          this->gripEffortThreshold << " Nm." << std::endl;
  }

  if (this->gripEffortThreshold <= 0)
  {
    gzerr << "GazeboGraspFix: <grip_effort_threshold> must be positive." <<
          std::endl;
    return;
  }

  sdf::ElementPtr attachDelayElem = GetOptionalElement(_sdf, "attach_delay");
  if (!attachDelayElem)
  {
    this->attachDelay = DEFAULT_ATTACH_DELAY;
    gzmsg << "GazeboGraspFix: Using default attach_delay " <<
          this->attachDelay << " s." << std::endl;
  }
  else
  {
    this->attachDelay = attachDelayElem->Get<double>();
    gzmsg << "GazeboGraspFix: Using attach_delay " <<
          this->attachDelay << " s." << std::endl;
  }

  if (this->attachDelay < 0)
  {
    gzerr << "GazeboGraspFix: <attach_delay> must not be negative." <<
          std::endl;
    return;
  }

  sdf::ElementPtr detachDelayElem = GetOptionalElement(_sdf, "detach_delay");
  if (!detachDelayElem)
  {
    this->detachDelay = DEFAULT_DETACH_DELAY;
    gzmsg << "GazeboGraspFix: Using default detach_delay " <<
          this->detachDelay << " s." << std::endl;
  }
  else
  {
    this->detachDelay = detachDelayElem->Get<double>();
    gzmsg << "GazeboGraspFix: Using detach_delay " <<
          this->detachDelay << " s." << std::endl;
  }

  if (this->detachDelay < 0)
  {
    gzerr << "GazeboGraspFix: <detach_delay> must not be negative." <<
          std::endl;
    return;
  }

  sdf::ElementPtr updateRateElem = GetOptionalElement(_sdf, "update_rate");
  double _updateSecs;
  if (!updateRateElem)
  {
    gzmsg << "GazeboGraspFix: Using  " << DEFAULT_UPDATE_RATE <<
          " because no <updateRate_tag> element specified." << std::endl;
    _updateSecs = 1.0 / DEFAULT_UPDATE_RATE;
  }
  else
  {
    int _rate = updateRateElem->Get<int>();
    double _updateRate = _rate;
    _updateSecs = 1.0 / _updateRate;
    gzmsg << "GazeboGraspFix: Using update rate " << _rate << std::endl;
  }
  this->updateRate = common::Time(0, common::Time::SecToNano(_updateSecs));

  sdf::ElementPtr maxGripCountElem = GetOptionalElement(_sdf, "max_grip_count");
  if (!maxGripCountElem)
  {
    gzmsg << "GazeboGraspFix: Using  " << DEFAULT_MAX_GRIP_COUNT <<
          " because no <max_grip_count> element specified." << std::endl;
    this->maxGripCount = DEFAULT_MAX_GRIP_COUNT;
  }
  else
  {
    this->maxGripCount = maxGripCountElem->Get<int>();
    gzmsg << "GazeboGraspFix: Using max_grip_count "
          << this->maxGripCount << std::endl;
  }

  sdf::ElementPtr gripCountThresholdElem =
    GetOptionalElement(_sdf, "grip_count_threshold");
  if (!gripCountThresholdElem)
  {
    this->gripCountThreshold = floor(this->maxGripCount / 2.0);
    gzmsg << "GazeboGraspFix: Using  " << this->gripCountThreshold <<
          " because no <grip_count_threshold> element specified." <<
          std::endl;
  }
  else
  {
    this->gripCountThreshold = gripCountThresholdElem->Get<int>();
    gzmsg << "GazeboGraspFix: Using grip_count_threshold " <<
          this->gripCountThreshold << std::endl;
  }

  this->effortWindow = 0.3;
  sdf::ElementPtr effortWindowElem = GetOptionalElement(_sdf, "effort_window");
  if (effortWindowElem)
    this->effortWindow = effortWindowElem->Get<double>();
  gzmsg << "GazeboGraspFix: Using effort_window " << this->effortWindow << " s." << std::endl;

  if (_sdf->HasElement("release_tolerance"))
  {
    gzwarn << "GazeboGraspFix: <release_tolerance> is obsolete and ignored; " <<
              "release is now based on gripper effort." << std::endl;
  }

  // will contain all names of collision entities involved from all arms
  std::vector<std::string> collisionNames;

  sdf::ElementPtr armElem = GetOptionalElement(_sdf, "arm");
  if (!armElem)
  {
    gzerr << "GazeboGraspFix: Cannot load the GazeboGraspFix without any <arm> declarations"
          << std::endl;
    return;
  }
  // add all arms:
  for (; armElem != NULL; armElem = armElem->GetNextElement("arm"))
  {
    sdf::ElementPtr armNameElem = GetOptionalElement(armElem, "arm_name");
    sdf::ElementPtr handLinkElem = GetOptionalElement(armElem, "palm_link");
    sdf::ElementPtr fingerLinkElem = GetOptionalElement(armElem, "gripper_link");

    if (!handLinkElem || !fingerLinkElem || !armNameElem)
    {
      gzerr << "ERROR: GazeboGraspFix: Cannot use a GazeboGraspFix arm because "
            << "not all of <arm_name>, <palm_link> and <gripper_link> elements specified in URDF/SDF. Skipping."
            << std::endl;
      continue;
    }

    std::string armName = armNameElem->Get<std::string>();
    std::string palmName = handLinkElem->Get<std::string>();

    physics::JointPtr forceJoint;
    sdf::ElementPtr forceJointElem = GetOptionalElement(armElem, "gripper_joint");
    if (forceJointElem)
    {
      const std::string forceJointName = forceJointElem->Get<std::string>();
      forceJoint = _parent->GetJoint(forceJointName);
      if (!forceJoint)
      {
        gzerr << "GazeboGraspFix: Gripper joint '" << forceJointName <<
              "' was not found for arm '" << armName << "'. Skipping." <<
              std::endl;
        continue;
      }
    }
    else
    {
      // MobiPick prefixes joint names with its robot namespace. Match the
      // stable Robotiq joint suffix so the plugin works with any tf_prefix.
      const std::string forceJointSuffix = "gripper_finger_joint";
      const physics::Joint_V modelJoints = _parent->GetJoints();
      for (physics::Joint_V::const_iterator jointIt = modelJoints.begin();
           jointIt != modelJoints.end(); ++jointIt)
      {
        const std::string jointName = (*jointIt)->GetName();
        if ((jointName.size() >= forceJointSuffix.size()) &&
            (jointName.compare(jointName.size() - forceJointSuffix.size(),
                               forceJointSuffix.size(), forceJointSuffix) == 0))
        {
          if (forceJoint)
          {
            gzerr << "GazeboGraspFix: More than one joint ends in '" <<
                  forceJointSuffix << "'. Add <gripper_joint> to arm '" <<
                  armName << "'. Skipping." << std::endl;
            forceJoint.reset();
            break;
          }
          forceJoint = *jointIt;
        }
      }
      if (!forceJoint)
      {
        gzerr << "GazeboGraspFix: Could not auto-detect a joint ending in '" <<
              forceJointSuffix << "' for arm '" << armName << "'. Skipping." <<
              std::endl;
        continue;
      }
      gzmsg << "GazeboGraspFix: Auto-detected gripper joint " <<
            forceJoint->GetName() << " for arm " << armName << "." << std::endl;
    }

    // collect all finger names:
    std::vector<std::string> fingerLinkNames;
    for (; fingerLinkElem != NULL;
         fingerLinkElem = fingerLinkElem->GetNextElement("gripper_link"))
    {
      std::string linkName = fingerLinkElem->Get<std::string>();
      fingerLinkNames.push_back(linkName);
    }

    // add new gripper
    if (grippers.find(armName) != grippers.end())
    {
      gzerr << "GazeboGraspFix: Arm named " << armName <<
            " was already added, cannot add it twice." << std::endl;
    }
    GazeboGraspGripper &gripper = grippers[armName];
    std::map<std::string, physics::CollisionPtr> _collisions;
    if (!gripper.Init(_parent, armName, palmName, fingerLinkNames, forceJoint,
                      disableCollisionsOnAttach, _collisions))
    {
      gzerr << "GazeboGraspFix: Could not initialize arm " << armName << ". Skipping."
            << std::endl;
      grippers.erase(armName);
      continue;
    }
    // add all the grippers's collision elements
    for (std::map<std::string, physics::CollisionPtr>::iterator collIt =
           _collisions.begin();
         collIt != _collisions.end(); ++collIt)
    {
      const std::string &collName = collIt->first;
      //physics::CollisionPtr& coll=collIt->second;
      std::map<std::string, std::string>::iterator collIter = this->collisions.find(
            collName);
      if (collIter !=
          this->collisions.end())   //this collision was already added before
      {
        gzwarn << "GazeboGraspFix: Adding Gazebo collision link element " << collName <<
               " multiple times, the grasp plugin may not work properly" << std::endl;
        continue;
      }
      gzmsg << "GazeboGraspFix: Adding collision scoped name " << collName <<
            std::endl;
      this->collisions[collName] = armName;
      collisionNames.push_back(collName);
    }
  }

  if (grippers.empty())
  {
    gzerr << "ERROR: GazeboGraspFix: Cannot use a GazeboGraspFix because "
          << "no arms were configured successfully. Plugin will not work." << std::endl;
    return;
  }

  // ++++++++++++ start up things +++++++++++++++

  physics::PhysicsEnginePtr physics = GetPhysics(this->world);
  this->node->Init(gazebo::GetName(*(this->world)));
  physics::ContactManager *contactManager = physics->GetContactManager();
  contactManager->PublishContacts();  // TODO not sure this is required?

  filter_name = model->GetScopedName();
  std::string topic = contactManager->CreateFilter(filter_name,
                      collisionNames);
  if (!this->contactSub)
  {
    gzmsg << "Subscribing contact manager to topic " << topic << std::endl;
    bool latching = false;
    this->contactSub = this->node->Subscribe(topic, &GazeboGraspFix::OnContact,
                       this, latching);
  }

  gzmsg << "Advertising grasping events on topic grasp_events" << std::endl;
  this->eventsPub = this->node->Advertise<msgs::GraspEvent>("~/grasp_events");

  update_connection = event::Events::ConnectWorldUpdateEnd(boost::bind(
                        &GazeboGraspFix::OnUpdate, this));
}

////////////////////////////////////////////////////////////////////////////////
class GazeboGraspFix::ObjectContactInfo
{
  public:

    ObjectContactInfo():
      maxGripperContactCnt(0),
      leftFingerContact(false),
      rightFingerContact(false)
    {}

    // all grippers involved in the process, along with
    // a number counting the number of contact points with the
    // object per gripper
    std::map<std::string, int> grippersInvolved;

    // maximum number of contacts of *any one* gripper
    // (any in grippersInvolved)
    int maxGripperContactCnt;

    // the gripper for maxGripperContactCnt
    std::string maxContactGripper;

    // MobiPick's two finger sides must both contact the object. This is more
    // reliable than comparing noisy Gazebo contact-wrench directions.
    bool leftFingerContact;
    bool rightFingerContact;
};

////////////////////////////////////////////////////////////////////////////////
bool GazeboGraspFix::IsGripperLink(const std::string &linkName,
                                   std::string &gripperName) const
{
  for (std::map<std::string, GazeboGraspGripper>::const_iterator it =
         grippers.begin(); it != grippers.end(); ++it)
  {
    if (it->second.hasLink(linkName))
    {
      gripperName = it->first;
      return true;
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
std::map<std::string, std::string> GazeboGraspFix::GetAttachedObjects() const
{
  std::map<std::string, std::string> ret;
  for (std::map<std::string, GazeboGraspGripper>::const_iterator it =
         grippers.begin(); it != grippers.end(); ++it)
  {
    const std::string &gripperName = it->first;
    const GazeboGraspGripper &gripper = it->second;
    if (gripper.isObjectAttached())
    {
      ret[gripper.attachedObject()] = gripperName;
    }
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
bool GazeboGraspFix::ObjectAttachedToGripper(const ObjectContactInfo
    &objContInfo, std::string &attachedToGripper) const
{
  for (std::map<std::string, int>::const_iterator gripInvIt =
         objContInfo.grippersInvolved.begin();
       gripInvIt != objContInfo.grippersInvolved.end(); ++gripInvIt)
  {
    const std::string &gripperName = gripInvIt->first;
    if (ObjectAttachedToGripper(gripperName, attachedToGripper))
    {
      return true;
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
bool GazeboGraspFix::ObjectAttachedToGripper(const std::string &gripperName,
    std::string &attachedToGripper) const
{
  std::map<std::string, GazeboGraspGripper>::const_iterator gIt = grippers.find(
        gripperName);
  if (gIt == grippers.end())
  {
    gzerr << "GazeboGraspFix: Inconsistency, gripper " << gripperName <<
          " not found in GazeboGraspFix grippers" << std::endl;
    return false;
  }
  const GazeboGraspGripper &gripper = gIt->second;
  // gzmsg<<"Gripper "<<gripperName<<" is involved in the grasp"<<std::endl;
  if (gripper.isObjectAttached())
  {
    attachedToGripper = gripperName;
    return true;
  }
  return false;
}


////////////////////////////////////////////////////////////////////////////////
/**
 * Helper class to encapsulate a collision information.
 * One contact has two bodies, and only
 * the ones where one of the bodies is a gripper link are considered.
 * Each contact consists of a *list* of forces with their own origin/position each
 * (e.g. when the object and gripper are colliding at several places).
 * The averages of each contact's force vectors along with their origins are
 * *accumulated* in the given Vector3 \e pos and \eforce objects.
 * The number of additions is stored in \e sum.
 * This is to get the average force application over time between link and object.
 *
 * \author Jennifer Buehler
 */
class GazeboGraspFix::CollidingPoint
{
  public:
    CollidingPoint(): sum(0) {}
    CollidingPoint(const CollidingPoint &o):
      gripperName(o.gripperName),
      collLink(o.collLink),
      collObj(o.collObj),
      force(o.force),
      pos(o.pos),
      objPos(o.objPos),
      sum(o.sum) {}

    // Name of the gripper/arm involved in contact point
    // This is not the specific link, but the name of the
    // whole gripper
    std::string gripperName;

    // the collision
    physics::CollisionPtr collLink, collObj;

    // average force vector of the colliding point
    GzVector3 force;

    // position (relative to reference frame of gripper
    // collision surface) where the contact happens on collision surface
    GzVector3 pos;

    // position (relative to reference frame of *gripper* collision surface)
    // where the object center is located during collision.
    GzVector3 objPos;

    // sum of force and pose (they are actually summed
    // up from several contact points).
    // Divide both \e force and \e pos by this to obtain average
    int sum;
};

////////////////////////////////////////////////////////////////////////////////
double GazeboGraspFix::GripEffort(const std::string &gripperName) const
{
  std::map<std::string, std::deque<std::pair<double, double> > >::const_iterator it =
    this->effortSamples.find(gripperName);
  if (it == this->effortSamples.end() || it->second.empty())
    return 0.0;
  double best = it->second.front().second;
  for (std::deque<std::pair<double, double> >::const_iterator sIt = it->second.begin();
       sIt != it->second.end(); ++sIt)
    best = std::max(best, sIt->second);
  return best;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboGraspFix::OnUpdate()
{
#if GAZEBO_MAJOR_VERSION >= 8
  const double sampleTime = this->world->SimTime().Double();
#else
  const double sampleTime = this->world->GetSimTime().Double();
#endif
  // Sample the finger effort in every physics step: single readings drop to 0 while squeezing (#126).
  for (std::map<std::string, GazeboGraspGripper>::iterator gIt = grippers.begin(); gIt != grippers.end(); ++gIt)
  {
    std::deque<std::pair<double, double> > &samples = this->effortSamples[gIt->first];
    const double effort = gIt->second.gripEffort();
    if (effort != 0.0 || samples.empty())
      samples.push_back(std::make_pair(sampleTime, effort));
    while (!samples.empty() && (samples.front().first < sampleTime - this->effortWindow ||
                                samples.front().first > sampleTime))  // sim time reset
      samples.pop_front();
  }

  if ((common::Time::GetWallTime() - this->prevUpdateTime) < this->updateRate)
    return;

  // First, copy all contact data into a local structure so contact processing
  // does not hold the mutex while evaluating the grasp.
  this->mutexContacts.lock();
  std::map<std::string, std::map<std::string, CollidingPoint> > contPoints(
    this->contacts);
  this->contacts.clear(); // clear so it can be filled anew by OnContact().
  this->mutexContacts.unlock();

#if GAZEBO_MAJOR_VERSION >= 8
  const common::Time simTime = this->world->SimTime();
#else
  const common::Time simTime = this->world->GetSimTime();
#endif

  // contPoints now contains CollidingPoint objects for each *object* and *link*.

  // Iterate through all contact points to gather all summed forces
  // (and other useful information) for all the objects (so we have all forces on one object).
  std::map<std::string, std::map<std::string, CollidingPoint> >::iterator objIt;
  std::map<std::string, ObjectContactInfo> objectContactInfo;

  for (objIt = contPoints.begin(); objIt != contPoints.end(); ++objIt)
  {
    std::string objName = objIt->first;
    // Objects with several collisions (e.g. the banana, 7 boxes, #126) are gripped when the two fingers touch
    // different collisions of the same link: accumulate per link, under the link's first collision name
    // (lexicographic, so the key is stable between updates; attaching it fixes the whole link).
    if (!objIt->second.empty() && objIt->second.begin()->second.collObj &&
        objIt->second.begin()->second.collObj->GetLink())
    {
      const physics::Collision_V linkCollisions =
        objIt->second.begin()->second.collObj->GetLink()->GetCollisions();
      for (physics::Collision_V::const_iterator cIt = linkCollisions.begin(); cIt != linkCollisions.end(); ++cIt)
        if ((*cIt)->GetScopedName() < objName) objName = (*cIt)->GetScopedName();
    }

    // create new entry in accumulated results map and get reference to fill in:
    ObjectContactInfo &objContInfo = objectContactInfo[objName];

    // for all links colliding with this object...
    std::map<std::string, CollidingPoint>::iterator lIt;
    for (lIt = objIt->second.begin(); lIt != objIt->second.end(); ++lIt)
    {
      std::string linkName = lIt->first;
      CollidingPoint &collP = lIt->second;
      if (linkName.find("gripper_left_") != std::string::npos)
        objContInfo.leftFingerContact = true;
      if (linkName.find("gripper_right_") != std::string::npos)
        objContInfo.rightFingerContact = true;
      // insert the gripper (if it doesn't exist yet) and increase contact counter
      int &gContactCnt = objContInfo.grippersInvolved[collP.gripperName];
      gContactCnt++;
      int &_maxGripperContactCnt = objContInfo.maxGripperContactCnt;
      if (gContactCnt > _maxGripperContactCnt)
      {
        _maxGripperContactCnt = gContactCnt;
        objContInfo.maxContactGripper = collP.gripperName;
      }
    }
  }

  // ++++++++++++++++++++ Handle Attachment  +++++++++++++++++++++++++++++++

  // collect of all objects which are found to be "gripped" at the current stage.
  // if they are gripped, increase the grip counter. If the grip count exceeds the
  // threshold, attach the object to the gripper which has most contact points with the
  // object.
  std::set<std::string> grippedObjects;
  for (std::map<std::string, ObjectContactInfo>::iterator ocIt =
         objectContactInfo.begin();
       ocIt != objectContactInfo.end(); ++ocIt)
  {
    const std::string &objName = ocIt->first;
    const ObjectContactInfo &objContInfo = ocIt->second;

    // gzmsg<<"Number applied forces on "<<objName<<": "<<objContInfo.appliedForces.size()<<std::endl;
  
    // TODO: remove this test print, for issue #26 ------------------- 
#if 0
    physics::CollisionPtr objColl =
      boost::dynamic_pointer_cast<physics::Collision>(GetEntityByName(world, objName));
    if (objColl && objColl->GetLink())
    {
      auto linVel = GetWorldVelocity(objColl->GetLink());
      gzmsg << "Velocity for link " << objColl->GetLink()->GetName()
        << " (collision name " << objName << "): " << linVel
        << ", absolute val " << GetLength(linVel) << std::endl;
    }
#endif
    // ------------------- 

    if (!objContInfo.leftFingerContact || !objContInfo.rightFingerContact)
      continue;

    const std::string &graspingGripperName = objContInfo.maxContactGripper;
    std::map<std::string, GazeboGraspGripper>::iterator gIt = grippers.find(
          graspingGripperName);
    if (gIt == grippers.end())
    {
      gzerr << "GazeboGraspFix: Inconsistency, gripper '" << graspingGripperName
            << "' not found while checking effort for object " << objName <<
            std::endl;
      continue;
    }
    GazeboGraspGripper &graspingGripper = gIt->second;
    if (GripEffort(graspingGripperName) < this->gripEffortThreshold)
      continue;

    // add to "gripped objects"
    grippedObjects.insert(objName);

    //gzmsg<<"Grasp Held: "<<objName<<" grip count: "<<this->gripCounts[objName]<<std::endl;

    int &counts = this->gripCounts[objName];
    if (counts < this->maxGripCount) ++counts;

    std::map<std::string, common::Time>::iterator startIt =
      this->gripStartTimes.find(objName);
    if (startIt == this->gripStartTimes.end())
    {
      startIt = this->gripStartTimes.insert(
        std::make_pair(objName, simTime)).first;
    }
    this->gripCandidates[objName] = graspingGripperName;

    // Let contact physics establish a stable grasp before adding the fixed joint.
    const double grippingSecs = (simTime - startIt->second).Double();
    if ((counts <= this->gripCountThreshold) ||
        (grippingSecs < this->attachDelay))
      continue;

    //gzmsg<<"GRIPPING "<<objName<<", grip count "<<counts<<" (threshold "<<this->gripCountThreshold<<")"<<std::endl;

    // find out if any of the grippers involved in the grasp is already grasping the object.
    // If there is no such gripper, attach it to the gripper which has most contact points.
    std::string attachedToGripper;
    bool isAttachedToGripper = ObjectAttachedToGripper(objContInfo,
                               attachedToGripper);
    if (isAttachedToGripper)
    {
      // the object is already attached to a gripper, so it does not need to be attached.
      // gzmsg << "GazeboGraspFix has found that object "<<
      //     gripper.attachedObject()<<" is already attached to gripper "<<gripperName;
      continue;
    }

    if (graspingGripper.isObjectAttached())
    {
      gzerr << "GazeboGraspFix has found that object " <<
            graspingGripper.attachedObject() << " is already attached to gripper " <<
            graspingGripperName << ", so can't grasp '" << objName << "'!" << std::endl;
      continue;
    }

    gzmsg << "GazeboGraspFix: Attaching " << objName << " to gripper " <<
          graspingGripperName << "." << std::endl;

    if (!graspingGripper.HandleAttach(objName))
    {
      gzerr << "GazeboGraspFix: Could not attach object " << objName << " to gripper "
            << graspingGripperName << std::endl;
    }
    else
    {
      this->OnAttach(objName, graspingGripperName);
      this->gripCandidates.erase(objName);
      this->gripStartTimes.erase(objName);
    }
  }  // for all objects



  // ++++++++++++++++++++ Handle Detachment  +++++++++++++++++++++++++++++++
  std::map<std::string, std::string> attachedObjects = GetAttachedObjects();

  // Decay the attachment qualification state when contact or squeeze is lost.
  std::map<std::string, int>::iterator gripCntIt;
  for (gripCntIt = this->gripCounts.begin(); gripCntIt != this->gripCounts.end();
       ++gripCntIt)
  {

    const std::string &objName = gripCntIt->first;

    if (grippedObjects.find(objName) != grippedObjects.end())
    {
      // this object is one we just detected as "gripped", so no need to check for releasing it...
      // gzmsg<<"NOT considering "<<objName<<" for detachment."<<std::endl;
      continue;
    }

    // Contact reports can flicker while the controller is still squeezing.
    // Keep the attachment timer alive in that case; a later contact update can
    // complete the attachment after physics has had the configured delay.
    bool squeezingCandidate = false;
    std::map<std::string, std::string>::const_iterator candidateIt =
      this->gripCandidates.find(objName);
    if (candidateIt != this->gripCandidates.end())
    {
      std::map<std::string, GazeboGraspGripper>::const_iterator candidateGripperIt =
        grippers.find(candidateIt->second);
      squeezingCandidate = (candidateGripperIt != grippers.end()) &&
        (GripEffort(candidateIt->second) >= this->gripEffortThreshold);
    }
    if (squeezingCandidate)
      continue;

    this->gripStartTimes.erase(objName);
    this->gripCandidates.erase(objName);

    // gzmsg<<"NOT-GRIPPING "<<objName<<", grip count "<<gripCntIt->second<<" (threshold "<<this->gripCountThreshold<<")"<<std::endl;

    if (gripCntIt->second > 0) --(gripCntIt->second);

  }

  // A fixed joint can unload the fingertip contacts, so release must use the
  // master finger joint effort directly rather than contact force or distance.
  for (std::map<std::string, std::string>::const_iterator attIt =
         attachedObjects.begin(); attIt != attachedObjects.end(); ++attIt)
  {
    const std::string &objName = attIt->first;
    const std::string &graspingGripperName = attIt->second;
    std::map<std::string, GazeboGraspGripper>::iterator gripperIt =
      grippers.find(graspingGripperName);
    if (gripperIt == grippers.end())
    {
      gzerr << "GazeboGraspFix: Inconsistency: Gazebo gripper '" <<
            graspingGripperName << "' not found when checking for detachment" << std::endl;
      continue;
    }
    GazeboGraspGripper &graspingGripper = gripperIt->second;
    const double gripEffort = GripEffort(graspingGripperName);
    if (gripEffort >= this->gripEffortThreshold)
    {
      this->lowEffortStartTimes.erase(objName);
      continue;
    }

    std::map<std::string, common::Time>::iterator lowEffortIt =
      this->lowEffortStartTimes.find(objName);
    if (lowEffortIt == this->lowEffortStartTimes.end())
    {
      lowEffortIt = this->lowEffortStartTimes.insert(
        std::make_pair(objName, simTime)).first;
    }
    if ((simTime - lowEffortIt->second).Double() < this->detachDelay)
      continue;

    gzmsg << "GazeboGraspFix: Detaching " << objName << " from gripper " <<
          graspingGripperName << " because gripper effort " << gripEffort <<
          " Nm fell below " << this->gripEffortThreshold << " Nm." << std::endl;
    graspingGripper.HandleDetach(objName);
    this->OnDetach(objName, graspingGripperName);
    this->gripCounts[objName] = 0;
    this->gripStartTimes.erase(objName);
    this->gripCandidates.erase(objName);
    this->lowEffortStartTimes.erase(objName);
  }

  this->prevUpdateTime = common::Time::GetWallTime();
}

////////////////////////////////////////////////////////////////////////////////
void GazeboGraspFix::OnContact(const ConstContactsPtr &_msg)
{
  //gzmsg<<"CONTACT! "<<std::endl;//<<_contact<<std::endl;
  // for all contacts...
  for (int i = 0; i < _msg->contact_size(); ++i)
  {
    physics::CollisionPtr collision1 =
      boost::dynamic_pointer_cast<physics::Collision>(
        gazebo::GetEntityByName(this->world, _msg->contact(i).collision1()));
    physics::CollisionPtr collision2 =
      boost::dynamic_pointer_cast<physics::Collision>(
        gazebo::GetEntityByName(this->world, _msg->contact(i).collision2()));

    if ((collision1 && !collision1->IsStatic()) && (collision2
        && !collision2->IsStatic()))
    {
      std::string name1 = collision1->GetScopedName();
      std::string name2 = collision2->GetScopedName();

      //gzmsg<<"OBJ CONTACT! "<<name1<<" / "<<name2<<std::endl;
      int count = _msg->contact(i).position_size();

      // Check to see if the contact arrays all have the same size.
      if ((count != _msg->contact(i).normal_size()) ||
          (count != _msg->contact(i).wrench_size()) ||
          (count != _msg->contact(i).depth_size()))
      {
        gzerr << "GazeboGraspFix: Contact message has invalid array sizes\n" <<
              std::endl;
        continue;
      }

      std::string collidingObjName, collidingLink, gripperOfCollidingLink;
      physics::CollisionPtr linkCollision;
      physics::CollisionPtr objCollision;

      physics::Contact contact;
      contact = _msg->contact(i);

      if (contact.count < 1)
      {
        std::cerr << "ERROR: GazeboGraspFix: Not enough forces given for contact of ."
                  << name1 << " / " << name2 << std::endl;
        continue;
      }

      // all force vectors which are part of this contact
      std::vector<GzVector3> force;

      // find out which part of the colliding entities is the object, *not* the gripper,
      // and copy all the forces applied to it into the vector 'force'.
      std::map<std::string, std::string>::const_iterator gripperCollIt =
        this->collisions.find(name2);
      if (gripperCollIt != this->collisions.end())
      {
        // collision 1 is the object
        collidingObjName = name1;
        collidingLink = name2;
        linkCollision = collision2;
        objCollision = collision1;
        gripperOfCollidingLink = gripperCollIt->second;
        for (int k = 0; k < contact.count; ++k)
          force.push_back(contact.wrench[k].body1Force);
      }
      else if ((gripperCollIt = this->collisions.find(name1)) !=
               this->collisions.end())
      {
        // collision 2 is the object
        collidingObjName = name2;
        collidingLink = name1;
        linkCollision = collision1;
        objCollision = collision2;
        gripperOfCollidingLink = gripperCollIt->second;
        for (int k = 0; k < contact.count; ++k)
          force.push_back(contact.wrench[k].body2Force);
      }

      GzVector3 avgForce;
      // compute average/sum of the forces applied on the object
      for (int k = 0; k < force.size(); ++k)
      {
        avgForce += force[k];
      }
      avgForce /= force.size();

      GzVector3 avgPos;
      // compute center point (average pose) of all the origin positions of the forces appied
      for (int k = 0; k < contact.count; ++k) avgPos += contact.positions[k];
      avgPos /= contact.count;

      // now, get average pose relative to the colliding link
      GzPose3 linkWorldPose = gazebo::GetWorldPose(linkCollision->GetLink());

      // To find out the collision point relative to the Link's local coordinate system, first get the Poses as 4x4 matrices
      GzMatrix4 worldToLink = gazebo::GetMatrix(linkWorldPose);

      // We can assume that the contact has identity rotation because we don't care about its orientation.
      // We could always set another rotation here too.
      GzMatrix4 worldToContact = gazebo::GetMatrix(avgPos);

      // now, worldToLink * contactInLocal = worldToContact
      // hence, contactInLocal = worldToLink.Inv * worldToContact
      GzMatrix4 worldToLinkInv = worldToLink.Inverse();
      GzMatrix4 contactInLocal = worldToLinkInv * worldToContact;
      GzVector3 contactPosInLocal = gazebo::GetPos(contactInLocal);

      //gzmsg<<"---------"<<std::endl;
      //gzmsg<<"CNT in loc: "<<contactPosInLocal.x<<","<<contactPosInLocal.y<<","<<contactPosInLocal.z<<std::endl;

      /*GzVector3 sDiff=avgPos-linkWorldPose.pos;
      gzmsg<<"SIMPLE trans: "<<sDiff.x<<","<<sDiff.y<<","<<sDiff.z<<std::endl;
      gzmsg<<"coll world pose: "<<linkWorldPose.pos.x<<", "<<linkWorldPose.pos.y<<", "<<linkWorldPose.pos.z<<std::endl;
      gzmsg<<"contact avg pose: "<<avgPos.x<<", "<<avgPos.y<<", "<<avgPos.z<<std::endl;

      GzVector3 lX=linkWorldPose.rot.GetXAxis();
      GzVector3 lY=linkWorldPose.rot.GetYAxis();
      GzVector3 lZ=linkWorldPose.rot.GetZAxis();

      gzmsg<<"World ori: "<<linkWorldPose.rot.x<<","<<linkWorldPose.rot.y<<","<<linkWorldPose.rot.z<<","<<linkWorldPose.rot.w<<std::endl;
      gzmsg<<"x axis: "<<lX.x<<","<<lX.y<<","<<lX.z<<std::endl;
      gzmsg<<"y axis: "<<lY.x<<","<<lY.y<<","<<lY.z<<std::endl;
      gzmsg<<"z axis: "<<lZ.x<<","<<lZ.y<<","<<lZ.z<<std::endl;*/

      // Get the pose of the object and compute it's relative position to
      // the collision surface.
      GzPose3 objWorldPose = gazebo::GetWorldPose(objCollision->GetLink());
      GzMatrix4 worldToObj = gazebo::GetMatrix(objWorldPose);

      GzMatrix4 objInLocal = worldToLinkInv * worldToObj;
      GzVector3 objPosInLocal = gazebo::GetPos(objInLocal);

      {
        boost::mutex::scoped_lock lock(this->mutexContacts);
        CollidingPoint &p =
          this->contacts[collidingObjName][collidingLink];  // inserts new entry if doesn't exist
        p.gripperName = gripperOfCollidingLink;
        p.collLink = linkCollision;
        p.collObj = objCollision;
        p.force += avgForce;
        p.pos += contactPosInLocal;
        p.objPos += objPosInLocal;
        p.sum++;
      }
      //gzmsg<<"Average force of contact= "<<avgForce.x<<", "<<avgForce.y<<", "<<avgForce.z<<" out of "<<force.size()<<" vectors."<<std::endl;
    }
  }
}

void GazeboGraspFix::OnAttach(const std::string &objectName,
                              const std::string &armName)
{
  msgs::GraspEvent event;
  event.set_arm(armName);
  event.set_object(objectName);
  event.set_attached(true);
  eventsPub->Publish(event);
}

void GazeboGraspFix::OnDetach(const std::string &objectName,
                              const std::string &armName)
{
  msgs::GraspEvent event;
  event.set_arm(armName);
  event.set_object(objectName);
  event.set_attached(false);
  eventsPub->Publish(event);
}
