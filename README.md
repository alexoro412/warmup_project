# warmup_project

## Drive in a Square

The goal here is just to get a robot to
drive in a square. I achieved this by having
the robot perform a sequence of moves with 
set velocities and durations.

I have a class called `MoveState` that represents
a preset movement with forward velocity, angular
velocity, and duration. I also created a `SquareDriver`
class that controls the robot. `SquareDriver.set_velocity`
sets the velocity via the `/cmd_vel` channel. `SquareDriver.run`
loops through the list of `MoveStates` and sets the velocity
accordingly.

(the robot starts moving a bit late in the recording)

![square](./square.gif)


## Person Follower

The goal here is to have to the robot follower 
a person/cylinder. I achieved this by reading in the 
LIDAR scan data, and using proportional control to 
have the robot both turn towards the person,
and drive towards them as well.

I have a `PersonFollower` class that encapsulates
all of my logic. The `set_velocity` function just sends
a message on `/cmd_vel` to set the velocity. The run
function starts the subscriber. `process_scan` is the 
callback to the subscriber; this function reads 
scan data and uses proportional control to steer
the robot towards the person. I also added a few conditionals
so the robot would only drive forward if it was
roughly pointing at the person, and so that it wouldn't 
continue to jiggle after it was pointed towards the person.

![person](./person.gif)

## Wall Follower

The goal here is to have the robot follow along the wall
in a square room. Originally, I had planned out a finite
state machine, where the robot would transition to the 
turning state in corners, and the following state
when next to a wall. That proved to be really difficult to
get working, so instead I went with a simpler setup. If the
robot detects something in front of it, it will turn to
avoid it. Otherwise, it will drive straight. This requires much less code, and works pretty well.

I have a `WallFollower` class that defines this behavior. It
is fairly similar to the previous classes, with `run`, `set_velocity`, and `process_scan`. The `process_scan` function reads
distances from a range of angles in the scan; this helps with
variation in scan data, and also helps the robot steer more away from the wall.

![wall](./wall.gif)

# Conclusion

The biggest challenge was getting the movements 
to be consistent when driving in a square. As I was tuning the velocities/durations,
I would encounter issues where the first two turns worked
fine, but the third was way off. Eventually, through some
trial-and-error, I got the turns well tuned. Another challenge
was dealing with Gazebo issues. When I was first writing my 
wall follower, it looked like it was following a non-existent wall. And then I saw it drive through a wall. I eventually
figured out that if I reset the world, I should move the walls
ever so slightly to get Gazebo to properly update their positions.

In the future, I would like to work on tuning the robots proportional controls, and potentially add integral/derivative control, so that it moves more smoothly. I would also like to eliminate some of my duplicated code, and ideally abstract out some of the common behaviors between the person and wall follower.

Takeaways:
1. Friction is very annoying. Sometimes the robot would just not turn enough, and then veer way off course. It is important to rely on sensors, not just timing, in order to make movements more robust.
2. Make sure to use rospy.sleep well. Originally, I was running a loop 100 times a second and updating the speed through that for my square driver. But by instead just sleeping for the duration of the movement, my controller became more accurate and simpler to maintain.
3. Keep it simple. For my wall follower I originally tried a much more complicated solution that just wasn't working. By sticking to a simpler solution, I got a working robot that was reasonably robust. And because it's simpler, it's easier to extend and improve this robots behavior.
