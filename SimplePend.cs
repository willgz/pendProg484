//============================================================================
// SimplePend.cs    Defines a class for simulating a simple pendulum
//============================================================================
using System;

namespace Sim
{
    public class SimplePend
    {
        private double len = 1.1; // pendulum length
        private double g = 9.81; // gravitational field strength
        int n = 2;                  // number of states
        private double[] x;         // array of states
        private double[] f;         // right side of equation
        private double[] xi;        // intermediate step
        private double[,] sl;       // slope

        //--------------------------------------------------------------------
        // constructor
        //--------------------------------------------------------------------
        public SimplePend()
        {
            x = new double[n];
            f = new double[n];
            xi = new double[n];
            sl = new double[n,4];

            x[0] = 1.0;
            x[1] = 0.0;
        }

        //--------------------------------------------------------------------
        // step: perform one integration step via Euler's Method or RK4 Method
        //--------------------------------------------------------------------
        public void step(double dt, string method = "EULER")
        {
            rhsFunc(x,f);
            for(int i=0; i<n;++i)
            {
                if(method == "EULER")
                {
                    x[i] = x[i] + f[i] * dt;
                }
                else if(method == "RK4")
                {
                    sl[i,0] = f[i];

                    xi[i] = x[i] + sl[i,0] * 0.5 * dt;
                    rhsFunc(xi,f);
                    sl[i,1] = f[i] + 0.5 * dt;

                    xi[i] = x[i] + sl[i,1] * 0.5 * dt;
                    rhsFunc(xi,f);
                    sl[i,2] = f[i] + 0.5 * dt;

                    xi[i] = x[i] + sl[i,2] * dt;
                    rhsFunc(xi,f);
                    sl[i,3] = f[i];

                    x[i] = x[i]+(sl[i,0]+2*sl[i,1]+2*sl[i,2]+sl[i,3])*dt/6;
                }
            }
        }

        //--------------------------------------------------------------------
        // rhsFunc: function to calculate rhs of pendulum ODEs
        //--------------------------------------------------------------------
        public void rhsFunc(double[] st, double[] ff)
        {
            ff[0] = st[1];
            ff[1] = -g/len * Math.Sin(st[0]);
        }

        //--------------------------------------------------------------------
        // Getters and setters
        //--------------------------------------------------------------------
        public double L
        {
            get{return(len);}

            set
            {
                if (value > 0.0)
                    len = value;
            }
        }
        public double G
        {
            get{return(g);}

            set
            {
                if (value >= 0.0)
                    g = value;
            }
        }
        public double theta
        {
            get{return(x[0]);}

            set {x[0] = value;}
        }
        public double thetaDot
        {
            get{return(x[1]);}

            set {x[1] = value;}
        }
    }
    
}