byte CIRCLE=0
byte LINE_STRIP=1
byte LINE_LIST=2
byte POLYGON=3
byte POINTS=4
byte FRAMES=5
byte TEXT=6

byte LINE_STRIP3D=7
byte LINE_LIST3D=8
byte POLYGON3D=9
byte POINTS3D=10
byte TEXT3D=11
byte CIRCLE3D=12

byte ADD=0
byte REMOVE=1

Header header
string ns		# namespace, used with id to form a unique id
int32 id          	# unique id within the namespace
int32 type        	# CIRCLE/LINE_STRIP/etc.
int32 action      	# ADD/REMOVE
geometry_msgs/Point position # used for CIRCLE/TEXT, 2D in pixel-coords
geometry_msgs/PointStamped position3D # used for 3DTEXT
geometry_msgs/PoseStamped pose # used for CIRCLE3D
float32 scale	 	# the diameter for a circle, etc.
float32 width	 	# the width for a line, etc.
std_msgs/ColorRGBA outline_color
byte filled		# whether to fill in the shape with color
std_msgs/ColorRGBA fill_color # color [0.0-1.0]
duration lifetime       # How long the object should last before being automatically deleted.  0 means forever
byte arc # used for CIRCLE3D
float32 angle # used for CIRCLE3D


geometry_msgs/Point[] points # used for LINE_STRIP/LINE_LIST/POLYGON/POINTS., 2D in pixel coords
PointArrayStamped points3D # used for 3DLINE_STRIP/3DLINE_LIST/3DPOLYGON/3DPOINTS
std_msgs/ColorRGBA[] outline_colors # a color for each line, point, etc.

string[] frames # used for FRAMES, tf names
string text             # used for TEXT, draw size of text is scale
bool left_up_origin     # draw text from left up origin
bool ratio_scale        #Use ratio respected to original image to specify scale and position
