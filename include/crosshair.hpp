#ifndef CROSSHAIR_HPP
#define CROSSHAIR_HPP

// Class to draw a crosshair on a ROS image
class Crosshair
{
    public:
        int x_crosshair;
        int y_crosshair;

        // Constructor
        explicit Crosshair(void);

        // Destructor
        ~Crosshair(void);


    private:

        void _draw_crosshair(void);

};


#endif