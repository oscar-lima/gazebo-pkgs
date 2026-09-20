#ifndef GAZEBO_GAZEBOGRASPFIX_H
#define GAZEBO_GAZEBOGRASPFIX_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <stdio.h>
#include <gazebo_grasp_plugin/GazeboGraspGripper.h>

namespace gazebo
{

/**
 * Inspired by gazebo::physics::Gripper, this plugin fixes an object which is grasped to the
 * robot hand to avoid problems with physics engines and to help the object staying in
 * the robot hand without slipping out.
 *
 * This is a *model* plugin, so you have to load the model plugin from the robot URDF:
 *
 * ```xml
 *   <gazebo>
 *       <plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">
 *          <arm>
 *              <arm_name>name-of-arm</arm_name>
 *              <palm_link> hand_link_name  </palm_link>
 *              <gripper_link> finger_index_link_1 </gripper_link>
 *              <gripper_link> finger_index_link_2 </gripper_link>
 *              <gripper_link> ... </gripper_link>
 *              <gripper_joint> gripper_finger_joint </gripper_joint>
 *           </arm>
 *           <update_rate>4</update_rate>
 *           <grip_count_threshold>4</grip_count_threshold>
 *           <max_grip_count>8</max_grip_count>
 *           <grip_effort_threshold>1.0</grip_effort_threshold>
 *           <attach_delay>1.0</attach_delay>
 *           <detach_delay>0.1</detach_delay>
 *           <disable_collisions_on_attach>false</disable_collisions_on_attach>
 *           <contact_topic>__default_topic__</contact_topic>
 *       </plugin>
 *   </gazebo>
 * ```
 *
 *    Description of the arguments:
 *
 *    - ``<arm>`` contains a collections of specification for each arm which can grasp one object (there may be several ``<arm>`` tags):
 *        - ``<arm_name>`` is the name of this arm. Has to be unique.
 *        - ``<palm_link>`` has to be the link to which the finger joints are attached.
 *        - ``<gripper_link>`` tags have to include -all- link names of the gripper/hand which are used to
 *              actively grasp objects (these are the links which determine whether a "grasp" exists according to
 *              above described criterion).
 *        - ``<gripper_joint>`` is the master finger joint whose applied effort indicates squeezing.
 *              If omitted, the plugin finds the unique joint ending in ``gripper_finger_joint``.
 *    - ``<update_rate>`` is the rate at which all contact points are checked against the "gripping criterion".
 *          Note that in-between such updates, existing contact points may be collected at
 *          a higher rate (the Gazebo world update rate). The ``update_rate`` is only the rate at
 *          which they are processed, which takes a bit of computation time, and therefore
 *          should be lower than the gazebo world update rate.
 *    - ``<grip_effort_threshold>`` is the minimum positive master finger joint effort in Nm required
 *           to attach and hold an object. MobiPick closes with positive effort and opens near zero.
 *    - ``<attach_delay>`` is the number of seconds for which the force-based grasp must be held
 *           continuously before the object is attached. It uses simulation time and defaults to 1 second.
 *    - ``<detach_delay>`` is the number of seconds for which the master joint effort must remain below
 *           the threshold before release. It filters one-cycle effort dropouts and defaults to 0.1 seconds.
 *    - ``<grip_count_threshold>`` is number of times in the update loop (running at update_rate) that an object has
 *            to be detected as "gripped" in order to attach the object.
 *            Adjust this with the update rate.
 *    - ``<max_grip_count>`` is the maximum number of a counter:
 *            At each update iteration (running at update_rate), if the "gripping criterion" is
 *            met for an object, a counter for this object is increased. ``max_grip_count`` is
 *            the maximum number recorded for an object. As soon as the counter goes beyond this
 *            number, the counter is stopped. As soon as the "gripping criterion" does not
 *            hold any more, the number will start to decrease again, (by 1 each time the object
 *            is detected as "not grasped" in an update iteration). So this counter is
 *            like a "buffer" which, when it is full, maintains the state, and when it is empty,
 *            again, the object is released.
 *            This should be at least double of ``grip_count_threshold``.
 *    - ``<disable_collisions_on_attach>`` can be used for the following:
 *            When an object is attached, collisions with it may be disabled, in case the
 *            robot still keeps wobbling.
 *    - ``<contact_topic>`` is the gazebo topic of contacts. Should normally be left at -\_\_default_topic\_\_-.
 *
 * Current limitations:
 *
 *  - Only one object can be attached per gripper.
 *  - Only partial support for an object cannot be gripped with two grippers (release condition may be
 *     triggered wrongly, or not at all, if two grippers are involved)
 *
 * \author Jennifer Buehler
 */
class GazeboGraspFix : public ModelPlugin
{
  public:
    GazeboGraspFix();
    GazeboGraspFix(physics::ModelPtr _model);
    virtual ~GazeboGraspFix();
  
  private:

    /**
     * Gets called just after the object has been attached to the palm link on \e armName
     */
    void OnAttach(const std::string &objectName,
                          const std::string &armName);
    /**
     * Gets called just after the object has been detached to the palm link on \e armName
     */
    void OnDetach(const std::string &objectName,
                          const std::string &armName);

    virtual void Init();
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    /**
     * Collects the current gripper contacts for each object. A MobiPick grasp
     * requires contact from both the left and right finger sides plus sufficient
     * positive effort at the master finger joint. When that is true, the number of "grip counts"
     * is increased for the holding arm (but grip counts will never exceed \e max_grip_count).
     * If the number of grip counts for this object exceeds \e grip_count_threshold,
     * the object is attached by calling GazeboGraspGripper::HandleAttach(object-name),
     * setting \e attached and \e attachedObjName.
     *
     * Attached objects are released when master-joint effort remains below
     * \e grip_effort_threshold for \e detach_delay. This deliberately does not use
     * distance to the original contact point, so opening releases compliant objects.
     */
    void OnUpdate();

    void InitValues();


    /**
     * Gets called upon detection of contacts.
     * A list of contacts is passed in \_msg. One contact has two bodies, and only
     * the ones where one of the bodies is a gripper link are considered.
     * Each contact consists of a *list* of forces with their own origin/position each
     * (e.g. when the object and gripper are colliding at several places).
     * The averages of each contact's force vectors along with their origins is computed.
     * This "average contact force/origin" for each contact is then added to the \e this->contacts map.
     * If an entry for this object/link pair already exists, the average force (and its origin)
     * is *added* to the existing force/origin, and the average count is increased. This is to get
     * the average force application over time between link and object.
     */
    void OnContact(const ConstContactsPtr &ptr);

    bool IsGripperLink(const std::string &linkName, std::string &gripperName) const;

    /**
     * return objects (key) and the gripper (value) to which it is attached
     */
    std::map<std::string, std::string> GetAttachedObjects() const;

    /**
     * Helper class to collect contact info per object.
     * Forward declaration here.
     */
    class ObjectContactInfo;

    /**
     * Helper function to determine if object attached to a gripper in ObjectContactInfo.
     */
    bool ObjectAttachedToGripper(const ObjectContactInfo &objContInfo,
                                 std::string &attachedToGripper) const;

    /**
     * Helper function to determine if object attached to this gripper
     */
    bool ObjectAttachedToGripper(const std::string &gripperName,
                                 std::string &attachedToGripper) const;


    // physics::ModelPtr model;
    // physics::PhysicsEnginePtr physics;
    physics::WorldPtr world;

    // sorted by their name, all grippers of the robot
    std::map<std::string, GazeboGraspGripper> grippers;

    event::ConnectionPtr update_connection;
    transport::NodePtr node;
    transport::PublisherPtr eventsPub;   // publisher of grasping events
    transport::SubscriberPtr contactSub; // subscriber to contact updates

    // minimum MobiPick master finger joint effort [Nm] required to attach/hold
    double gripEffortThreshold;

    // continuous force-qualified grasp duration [s] required before attachment
    double attachDelay;

    // continuous low-effort duration [s] required before detachment
    double detachDelay;

    // simulation time at which each uninterrupted force-qualified grasp started
    std::map<std::string, common::Time> gripStartTimes;

    // gripper associated with a pending object; retained through contact-message gaps
    std::map<std::string, std::string> gripCandidates;

    // simulation time at which low effort was first observed for an attached object
    std::map<std::string, common::Time> lowEffortStartTimes;

    // when an object is attached, collisions with it may be disabled, in case the
    // robot still keeps wobbling.
    bool disableCollisionsOnAttach;

    // all collisions per gazebo collision link (each entry
    // belongs to a physics::CollisionPtr element). The key
    // is the collision link name, the value is the gripper name
    // this collision link belongs to.
    std::map<std::string, std::string> collisions;


    /**
     * Helper class to encapsulate a collision information. Forward declaration here.
     */
    class CollidingPoint;

    // Contact forces sorted by object name the gripper collides with (first key)
    // and the link colliding (second key).
    std::map<std::string, std::map<std::string, CollidingPoint> > contacts;
    boost::mutex mutexContacts; //mutex protects contacts

    // Records how many subsequent update calls the grip on that object has been recorded
    // as "holding". Every loop, if a grip is not recorded, this number decreases.
    // When it reaches \e grip_count_threshold, it will be attached.
    // The number won't increase above max_grip_count once it has reached that number.
    std::map<std::string, int> gripCounts;

    // *maximum* number in \e gripCounts to be recorded.
    int maxGripCount;

    // number of recorded "grips" in the past (in gripCount) which, when it is exceeded, counts
    // as the object grasped, and when it is lower, as released.
    int gripCountThreshold;

    //nano seconds between two updates
    common::Time updateRate;

    //last time OnUpdate() was called
    common::Time prevUpdateTime;

    //ContactManager filter to be removed in destructor
    std::string filter_name;
};

}

#endif  // GAZEBO_GAZEBOGRASPFIX_H
