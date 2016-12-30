/*
# This is a simple A-star path planning algorithm that calculates the shortest path between the input position and output position.
# The input parameters are room width, start and target positions, step size.
# The output parameters are the path control points.
# Path selection parameters : Nuclear distance between the current position and goal , floor map value( Pref. Always move forward)
# Inital test with no fixed object was successfull. 
# To add fixed object, random numbers are generated and corresponding block is filled with largest probability to make the column to not be preferred
# Inital test with less number of fixed object was successfull but when the number of blocks are increased, the path starts rotating in the same control points.


Goal:
# To validate the path planning for 3D
# To develop a stable path planning algorithm to be coupled with the v-rep software so that the input is the workspace and start and goal configuration of EE.
# The step size is to be determined from the volume occupied by the EE.
*/



#include<iostream>
#include<math.h>
#include<iomanip>
#include<string>
#include<vector>
#include<fstream>
#include<cstdlib>
#include<ctime>



using namespace std;

/* 
	Idea is simple; We calculate a 2D path planning for the room structure;
	We convert the existing room to mathematical model of a 2D array;
	We locate the initial and final position in the array;
	Calculate equivalent floor id ((i*m)+j); initialize the vector with the size of calculated floor id.
	Determine the nearest neighbour and their distance from the target.
	Update them in ascending order to the vector. Hence easy to look for alternatives.
*/
struct data
{
	int id;// In other words the corresponding floor map
	int floor_map;// Respective value of floor map
	int n_id[8];// Neighbouring blocks floor map 
	double probab[8];// Corresponding Probability 
	int p_choice;// Probability Choice(0....7)
	
};
vector<data> my_data;


struct path
{
	int id;
	double xpos;
	double ypos;
	int p_c;// Value of p_choice
};
vector<path> my_path;



// Function Declaration
void sort(int *neigh, double *prob, int count);
void floor_map(double **room, int nx, int ny);
int my_count(double **room, int nx, int ny);
void collision(double **room, int nx, int ny, int x_init, int y_init, int x_fin, int y_fin, int n);

int main()
{
	// Variable Initialization
	double x, y; 		// Room Width
	double my_x, my_y; // Initial Position
	double tgt_x, tgt_y; // Final Position
	double dx, dy;		// Step Size
	int x_init, x_fin, y_init, y_fin;	// Initial and final matrix co ordinates;
	int nx, ny;			// Total number of cells in x and y direction;
	int x_n[3] = {-1,0,1};
	int y_n[3] = {-1,0,1};// To determine neighbours.
	int neigh[8] = {0}; // My neighbours floor id. Changes for every position.
	double prob[8] = {0};// Corresponding Probability.
	int x_temp = 0, y_temp = 0;
	int count = 0;
	int x_pos, y_pos;	//	For checking the boundary
	
	

	
	// Writing output to file
	//ofstream myfile;
	//myfile.open("prob.txt");
	ofstream f_map;
	f_map.open("floor_map.txt");
	// Path Determination Parameters
	double temp_x, temp_y;	// Variable to calculate the co ordinates and update the vector my_path
	int temp_x_floor, temp_y_floor;	// Variable to store the current calculated position for every iterative step.
	int my_floor, next_floor, target_floor; // Floor values taken as input.
	int n; 
	
	//Data Input for initialization. At later stage will be absorbed from software	
	cout<<"Enter the room size:x,y; inital position:my_x, my__y; target position: 	tgt_, tgt_y;"<<endl;
	cin>>x;
	cin>>y;
	cin>>my_x;
	cin>>my_y;
	cin>>tgt_x;
	cin>>tgt_y;
	cout<<"Enter the minimum step size:dx, dy"<<endl;
	cin>>dx;
	cin>>dy;
	cout<<"Great ! How many block you need to block in the cell"<<endl;
	cin>>n;
	// Calculation 1: Size of matrix and cell determination
	nx = ceil(x/dx);	// No of cells in x direction
	ny = ceil(y/dy);	// No of cells in y direction
	x_init = ceil(my_x/dx);	// Initial x position
	y_init = ceil(my_y/dy);	// Initial Y position
	x_fin = ceil(tgt_x/dx);	// Final x position	
	y_fin = ceil(tgt_y/dy);	// Final y position

	//Create a room for floor map study
	double** room = new double*[nx];
	for (int i = 0; i < nx; ++i)
	{
		room[i] = new double[ny];
		for(int j = 0; j < ny; ++j)
		{
			room[i][j] = 0;
		}
	}
	collision(room, nx, ny, x_init, y_init, x_fin, y_fin, n);
	
	
	room[x_init - 1][y_init - 1] = 1;

	floor_map(room, nx, ny);
	
	for(int i = 0; i < nx; i++)
	{
		for(int j = 0; j < ny; j++)
		{
			f_map<<room[i][j]<<",";
		}
		f_map<<endl;
	}
	
	for(int i = 0; i < (nx * ny); ++i)// Vector initialization to fix the size.
	{
		data local_data;
		local_data.id = i;
		local_data.floor_map = room[i/nx][i%ny];
		local_data.p_choice = 0;
		my_data.push_back(local_data);
	}
	
	// For every cell, we update a neighbour list with probability assigned to the target position distance.
	for(int i = 0; i < my_data.size(); ++i)
	{
		cout<<i<<endl;
		if ((my_data[i].id == i) && ( my_data[i].floor_map < (nx*ny)))
		{
			count = 0;
			x_temp = i / nx;
			y_temp = i % nx;
		//	cout<<"Success"<<my_data[i].id<<endl;
			//cout<<x_temp<<"\t"<<y_temp<<endl;
			// Create a local matrix(2*8) => floor id calculated and corresponding probability
			for(int ii = 0; ii < 3; ii++)
			{
				for(int jj = 0; jj < 3; jj++)
					
				{
					//cout<<ii<<"\t"<<jj<<endl;
					if((ii == 1) && (jj == 1))
					{
						//The execution is passed when the position is my current position.
						continue;
					}
					else
					{
						
						// These guys are not humans. They are machines. Let me not confuse them
						
						if ( x_temp ==0 || x_temp >= (nx-1) || y_temp == 0 || y_temp >= (ny-1))
						{
							x_pos = x_temp + x_n[ii];
							y_pos = y_temp + y_n[jj];	// My Neighbour started calculating
							
							if(x_pos < 0 || x_pos >=nx || y_pos < 0 || y_pos >= ny )
							{
									neigh[count] = - ((nx*ny) + (x_pos * nx) + y_pos);
									prob[count] = -((nx*ny) + (x_pos * nx) + y_pos);
									
							}
							else
							{
								neigh[count] = (x_pos * nx) + y_pos;
								prob[count] = sqrt(pow((x_pos - x_fin),2) + pow((y_pos - y_fin),2));// Distance based probability
							
								if (my_data[neigh[count]].floor_map > my_data[i].floor_map)
									prob[count] = prob[count]/2;			
						
								if ( my_data[neigh[count]].floor_map < my_data[i].floor_map)
									prob[count] = prob[count] * 2;
						
								if (my_data[neigh[count]].floor_map == my_data[(x_fin*nx)+y_fin].floor_map)
									prob[count] = prob[count]/4;
							}
							
						}
						else
						{
							x_pos = x_temp + x_n[ii];
							y_pos = y_temp + y_n[jj];	// My Neighbour started calculating
							neigh[count] = (x_pos * nx) + y_pos;
							prob[count] = sqrt(pow((x_pos - x_fin),2) + pow((y_pos - y_fin),2));// Distance based probability
							
							if (my_data[neigh[count]].floor_map > my_data[i].floor_map)
								prob[count] = prob[count]/2;			
						
							if ( my_data[neigh[count]].floor_map < my_data[i].floor_map)
								prob[count] = prob[count] * 2;
						
							if (my_data[neigh[count]].floor_map == my_data[(x_fin*nx)+y_fin].floor_map)
								prob[count] = prob[count]/4;
							
						}

						count++;
					}
				}
			}
			//cout<<"\n";
		}
		// Sorting neigh and prob array and store it in vector.
		sort(neigh, prob, count-1);

		for( int j = 0; j < 8; j++)
		{
			my_data[i].n_id[j] = neigh[j];
			my_data[i].probab[j] = prob[j];
		}
		//cout<<"Again Success"<<endl;
	}
	
	//Calculating the path.
	my_floor = (x_init * nx) + y_init;
	target_floor = (x_fin * nx) + y_fin;
	path local_path;
	local_path.xpos = (x_init/dx);
	local_path.ypos = (y_init/dy);
	my_path.push_back(local_path);
	int min = 0;
	int path_f[nx*ny];
	for(int i = 0; i < (nx*ny); i++)
		path_f[i] = 0;
	
	
	while (my_floor != target_floor)
	{
		min = 0;
		
		next_floor = my_data[my_floor].n_id[min];

		while ((my_data[my_floor].probab[min] < 0) || path_f[next_floor]>0)
		{
			min++;
			next_floor = my_data[my_floor].n_id[min];
			my_data[my_floor].p_choice = min;
			if ( min >= 8)
				cout<<" Sorry No Path"<<endl;
		}
		temp_x = next_floor / nx;
		temp_y = next_floor % nx;
		path local_path;
		local_path.xpos = (temp_x/dx);
		local_path.ypos = (temp_y/dy);
		local_path.p_c = next_floor;
		my_path.push_back(local_path);
		path_f[next_floor]++;
		cout<<(temp_y/dy)<<"\t"<<(temp_x/dx)<<endl;
		my_floor = next_floor;
	}
	
	

	
}
void sort(int *neigh, double *prob, int count)
{
	int i, j;
	int temp1;
	double temp;
	
	// Sorting Operation
	// Thanks to 'https://mathbits.com/MathBits/CompSci/Arrays/Exchange.htm'
	
	for(i = 0; i < (count-1); i++)
	{
		for(j = (i+1); j < count; j++)
		{
			if (prob[i] > prob[j])
			{
				temp = prob[i];
				prob[i] = prob[j];
				prob[j] = temp;
				
				// Index Arrays
				temp1 = neigh[i];
				neigh[i] = neigh[j];
				neigh[j] = temp1;
				
			}
		}
	}
	//return 0;
}


void floor_map(double **room, int nx, int ny)
{

	int x_part[3] = {-1,0,1};
	int y_part[3] = {-1,0,1};	// My Partners around me
	int update = 1;	// The update of floor map for every step

	int count = 0;	// to check if all the elements of matrix room are non zero
	count = my_count(room,nx,ny);// Function call to check if all the elements in a matrix are non zero 
	while (count != 0)// When matrix has zero elements
	{
		for(int i = 0; i<nx; i++)
		{
			for(int j = 0; j<ny; j++)
			{
				if((room[i][j] != 0) && (room[i][j] < (nx*ny)))// I locate a non zero element
				{					
					for(int k = 0; k<3; k++)
					{
						for(int l = 0; l<3; l++)
						{
							if (((i+(x_part[k]))>=0) && ((i+(x_part[k])) <nx) && ((j+(y_part[l])) >= 0) && ((j+(y_part[l]))< ny))// access the array x_part and y_part to access the neighbours. (Currently 2D)
							{
								if((room[i+x_part[k]][j+y_part[l]] ==0))// When a neighbour cell is zero
								{
									room[i+x_part[k]][j+y_part[l]] = room[i][j] + 1;// Add 1 to my cell and update the neighbour cell
								}
							}
						}
						}
				}
			}
		}
		count = my_count(room, nx, ny);// I again check the zeros in my matrix

	}
}
int my_count(double **room, int nx, int ny)// Function to check number of zeros in matrix
{
  int count = 0;
  for(int i = 0; i<nx; i++)
    for(int j = 0; j<ny; j++)
      if(room[i][j] == 0)
		count++;

  return count;
	 
}

void collision(double **room, int nx, int ny,int x_init,int y_init,int x_fin,int y_fin, int n)
{
	int x_t, y_t;

	int count = 0;
	srand(time(NULL));
	while(count < n)
	{
		x_t = rand() % nx + 0;
		y_t = rand() % ny + 0;
		
		if ( room [x_t][y_t] == 0)
		{
			if( (x_t != (x_fin - 1)) && (y_t != (y_fin - 1)))
			{
				room[x_t][y_t] = -((x_t * nx) + ny);
				count = count + 1;
			}
		}
	}
}

