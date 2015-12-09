import gurobi.GRBException;


public class Test {

	public static void main(String[] args) {
		int nDrivers = 1;
		int nPassengers = 1; 
		int[] seats = {2}; 
		float[] maxDist = {30};
		float[][] distance = {{0,5,10,1},{5,0,11,1},{10,11,0,12},{1,1,12,0}};
		
		GRBcarSharingSolver solver = new GRBcarSharingSolver(nDrivers, nPassengers, seats, maxDist, distance);
		try {
			solver.solveAndPrint();
		} catch (GRBException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
