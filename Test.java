import gurobi.GRBException;

public class Test {

	public static void main(String[] args) {
		int nDrivers = 2;
		int nPassengers = 3; 
		int[] seats = {1,1}; 
		float[] maxDist = {22,25};
		float[][] distance = {{0,7,100,5,10,200,6,30,15,70},
							  {7,0,100,3,4,150,14,40,4,30},
							  {100,100,0,100,100,20,100,50,100,60},
							  {5,3,100,0,5,100,8,35,6,50},
							  {10,4,100,5,0,100,19,50,2,50},
							  {200,150,20,100,100,0,200,100,200,50},
							  {6,14,100,8,19,200,0,20,17,30},
							  {30,40,50,35,50,100,20,0,20,20},
							  {15,4,100,6,2,200,17,20,0,25},
							  {70,30,60,50,50,50,30,20,25,0}};
		
		GRBcarSharingSolver solver = new GRBcarSharingSolver(nDrivers, nPassengers, seats, maxDist, distance);
		try {
			solver.solveAndPrint();
		} catch (GRBException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
