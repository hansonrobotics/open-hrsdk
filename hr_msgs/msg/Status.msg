bool has_current_target    # indicates if the robot arm is currently moved

float64 current_target_x   # x/y position of the current goal (only valid if has_current_target is true)
float64 current_target_y

bool has_current_pen_on_canvas   # indicates if the current_pen_on_canvas values are valid
float64 current_pen_on_canvas_x  # current x/y position of the pen on the canvas
float64 current_pen_on_canvas_y

float64 current_pen_x            # current x/y/z position of pen in 3d space
float64 current_pen_y
float64 current_pen_z

int32 queue                # amount of strokes that are still in the queue
