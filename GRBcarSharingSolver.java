import java.util.LinkedList;

import gurobi.*;

public class GRBcarSharingSolver {
	// Data members
	private int nDrivers;
	private int nPassengers;
	private int nNodes;
	private int[] seats;
	private int[] nodeLoad;
	private float[] maxDist;
	private float[][] distance;

	// Variables of the model
	private GRBVar[][][] travelFromTo;
	private GRBVar[][] carLoad;
	private GRBVar[] length;
	private GRBVar[][][] predecessor;
	
	// The model
	private GRBModel model;

	public GRBcarSharingSolver(int nDrivers, int nPassengers, int[] seats, float[] maxDist, float[][] distance) {
		// Assign data members
		this.nDrivers = nDrivers;
		this.nPassengers = nPassengers;
		this.nNodes = (nDrivers+nPassengers)*2;
		this.seats = seats;
		this.maxDist = maxDist;
		this.distance = distance;

		// Generate nodeLoads data
		this.nodeLoad = new int[nNodes];
		for (int i = 0; i < nPassengers; ++i) {
			nodeLoad[i] = 1;
			nodeLoad[nPassengers+i] = -1;
		}

		createModel();
	}

	private void createModel(){
		try {
			// Model
			GRBEnv env;

			env = new GRBEnv();

			this.model = new GRBModel(env);
			model.set(GRB.StringAttr.ModelName, "car sharing");

			// ======== VARIABLES ========= 

			// travelFromTo[i][j][k] == 1 if driver k travels from node i to node j
			travelFromTo = new GRBVar[nNodes][nNodes][nDrivers];
			for (int i = 0; i < nNodes; ++i) {
				for (int j = 0; j < nNodes; ++j) {
					for (int k = 0; k < nDrivers; ++k) {
						//						if (j < 2*nPassengers && j > 2*nPassengers+nDrivers) {
						travelFromTo[i][j][k] = model.addVar(0, 1, 0, GRB.BINARY,
								Integer.toString(k)+" travel From " + Integer.toString(i) + " To " + Integer.toString(j));
						//						} else {
						//							travelFromTo[i][j][k] = model.addVar(0, 0, 0, GRB.BINARY,
						//									Integer.toString(k)+" travel From " + Integer.toString(i) + " To " + Integer.toString(j));
						//						}
					}
				}
			}

			// predecessor[i][j][k] == 1 if node i is a predecessor of node j for a driver k
			predecessor = new GRBVar[nNodes][nNodes][nDrivers];
			for (int i = 0; i < nNodes; ++i) {
				for (int j = 0; j < nNodes; ++j) {
					for (int k = 0; k < nDrivers; ++k) {
						predecessor[i][j][k] = model.addVar(0, 1, 0, GRB.BINARY,
								Integer.toString(i)+" preceeds " + Integer.toString(j) + " for " + Integer.toString(k));
					}
				}
			}

			// carLoad[i][k] 
			carLoad = new GRBVar[nNodes][nDrivers];
			for (int i = 0; i < nNodes; ++i) {
				for (int k = 0; k < nDrivers; ++k) {
					int low_bound = Math.max(0, nodeLoad[i]);
					int up_bound = Math.min(seats[k], seats[k]+nodeLoad[i]);
					carLoad[i][k] = model.addVar(low_bound, up_bound, 0, GRB.INTEGER,
							Integer.toString(i)+"-" + Integer.toString(k) + " carLoad");
				}
			}

			// length[k]
			length = new GRBVar[nDrivers];
			for (int k = 0; k < nDrivers; ++k) {
				length[k] = model.addVar(0, maxDist[k], 0, GRB.CONTINUOUS, Integer.toString(k) + " length");
			}

			// Update model to integrate new variables
			model.update();


			// ========= CONSTRAINTS ==========

			GRBLinExpr lhs;

			// Defining the length of a driver's path
			for (int k = 0; k < nDrivers; ++k) {
				lhs = new GRBLinExpr();
				for (int i = 0; i < nNodes; ++i) {
					for (int j = 0; j < nNodes;  ++j) {
						lhs.addTerm(distance[i][j], travelFromTo[i][j][k]);
					}
				}
				model.addConstr(lhs, GRB.EQUAL, length[k], "length-"+Integer.toString(k));
			}

			// Each passenger can not be picked up more than once, 
			// but does not necessarily have to be picked up by a driver
			for (int i = 0; i < nPassengers; ++i) {
				lhs = new GRBLinExpr();
				for (int j = 0; j < nNodes; ++j) {
					for (int k = 0; k < nDrivers; ++k) {
						lhs.addTerm(1, travelFromTo[i][j][k]);
					}
				}
				model.addConstr(lhs, GRB.LESS_EQUAL, 1, Integer.toString(i)+"-Picked");
			}

			// Each passenger that has been picked up by a driver has to be dropped off by the same driver
			for (int i = 0; i < nPassengers; ++i) {
				for (int k = 0; k < nDrivers; ++k) {
					lhs = new GRBLinExpr();
					for (int j = 0; j < nNodes; ++j) {
						lhs.addTerm(1, travelFromTo[i][j][k]);
						lhs.addTerm(-1, travelFromTo[nPassengers+i][j][k]);
					}
					model.addConstr(lhs, GRB.EQUAL, 0, Integer.toString(i)+" drived by " + Integer.toString(k));
				}
			}

			//Each driver starts from his starting node and can’t go pick several passengers at the same time
			for (int k = 0; k < nDrivers; ++k) {
				lhs = new GRBLinExpr();
				for (int j = 0; j < nNodes; ++j) {
					lhs.addTerm(1, travelFromTo[2*nPassengers+k][j][k]);
				}
				model.addConstr(lhs, GRB.EQUAL, 1, "length-"+Integer.toString(k));
			}

			// A driver can’t go from a node to the same node
			//  &
			// Nobody can go through the starting node of a driver
			//	&
			// The driver's destination can't be left
			for (int i = 0; i < nNodes; ++i) {
				for (int j = 0; j < nNodes; ++j) {
					for (int k = 0; k < nDrivers; ++k) {
						if (i == j || (j >= nPassengers*2 && j < nPassengers*2+nDrivers) || i >= nPassengers*2+nDrivers) {
							model.addConstr(travelFromTo[i][j][k], GRB.EQUAL, 0, Integer.toString(i)+Integer.toString(j)+Integer.toString(k)+"impossible");
						}
					}
				}
			}

			// Each driver ends at his destination node and can’t arrive there from several nodes at the same time
			for (int k = 0; k < nDrivers; ++k) {
				lhs = new GRBLinExpr();
				for (int i = 0; i < nNodes; ++i) {
					lhs.addTerm(1, travelFromTo[i][2*nPassengers+nDrivers+k][k]);
				}
				model.addConstr(lhs, GRB.EQUAL, 1, Integer.toString(k)+" arrive at destination");
			}

			// Forcing the driver to leave a node where he picked or dropped a passenger
			for (int k = 0; k < nDrivers; ++k) {
				for(int i = 0; i < 2*nPassengers; ++i) {
					lhs = new GRBLinExpr();
					for (int j = 0; j < nNodes; ++j) {
						lhs.addTerm(1, travelFromTo[j][i][k]);
						lhs.addTerm(-1, travelFromTo[i][j][k]);
					}
					model.addConstr(lhs, GRB.EQUAL, 0, Integer.toString(i)+Integer.toString(k)+"continuous");
				}
			}

			// Consistency of the load variable
			for (int i = 0; i < nNodes; ++i) {
				for (int j = 0; j < nNodes; ++j) {
					for (int k = 0; k < nDrivers; ++k) {
						lhs = new GRBLinExpr();
						lhs.addTerm(1, carLoad[i][k]);
						lhs.addConstant(nodeLoad[j]);
						lhs.addConstant(-seats[k]);
						lhs.addTerm(1, travelFromTo[i][j][k]);
						model.addConstr(lhs, GRB.LESS_EQUAL, carLoad[j][k], Integer.toString(i)+Integer.toString(j)+Integer.toString(k)+"LoadConsistency");
					}
				}
			}

			// Constraints that ensure that a driver will always pick up a passenger before going through
			// the destination node of this same passenger
			for (int k = 0; k < nDrivers; ++k) {
				for (int i = 0; i < nPassengers*2; ++i) {
					// A drop-off node cannot be the predecessor of the corresponding pick-up node
					if (i < nPassengers) {
						model.addConstr(predecessor[i+nPassengers][i][k], GRB.EQUAL, 0, Integer.toString(i)+"dropImpossible");
					}

					for (int j = 0; j < nPassengers*2; ++j) {
						// If we visit node i before node j, the reverse is not possible
						lhs = new GRBLinExpr();
						lhs.addTerm(1, predecessor[i][j][k]);
						lhs.addTerm(1, predecessor[j][i][k]);
						model.addConstr(lhs, GRB.LESS_EQUAL, 1, Integer.toString(i)+Integer.toString(j)+Integer.toString(k)+"reverseImpossible");

						// If the driver k travels from i to j, then i precedes j
						model.addConstr(travelFromTo[i][j][k], GRB.LESS_EQUAL, predecessor[i][j][k], Integer.toString(i)+Integer.toString(j)+Integer.toString(k)+"predecessor");

						for (int p = 0; p < nPassengers*2; p++) {
							// If we visited node i before node j, a node p cannot both precede i and be a successor of j
							lhs = new GRBLinExpr();
							lhs.addTerm(1, predecessor[i][j][k]);
							lhs.addTerm(1, predecessor[p][i][k]);
							lhs.addTerm(1, predecessor[j][p][k]);
							model.addConstr(lhs, GRB.LESS_EQUAL, 2, Integer.toString(i)+Integer.toString(j)+Integer.toString(k)+"successor");

							// If driver k travels from i to j, then a node p cannot be between i and j
							lhs = new GRBLinExpr();
							lhs.addTerm(1, travelFromTo[i][j][k]);
							lhs.addTerm(1, predecessor[i][p][k]);
							lhs.addTerm(1, predecessor[p][j][k]);
							model.addConstr(lhs, GRB.LESS_EQUAL, 2, Integer.toString(i)+Integer.toString(j)+Integer.toString(k)+"bewteen");
						}
					}
				}
			}

			// ======== OBJECTIVE ==========

			GRBLinExpr obj = new GRBLinExpr();
			for (int i = 0; i < nNodes; ++i) {
				for (int j = 0; j < nNodes; ++j) {
					for (int k = 0; k < nDrivers; ++k) {
						obj.addTerm(1.0, travelFromTo[i][j][k]);
					}
				}
			}

			model.setObjective(obj, GRB.MAXIMIZE);	


		} catch (GRBException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("coucou");
		}
	}

	public void solveAndPrint() throws GRBException {

		model.optimize();
		int status = model.get(GRB.IntAttr.Status);
		if (status == GRB.Status.INF_OR_UNBD ||
				status == GRB.Status.INFEASIBLE  ||
				status == GRB.Status.UNBOUNDED     ) {
			System.out.println("The model cannot be solved "
					+ "because it is infeasible or unbounded");
			return;
		}
		if (status != GRB.Status.OPTIMAL ) {
			System.out.println("Optimization was stopped with status " + status);
			return;
		}

		// Print the solution
		System.out.println("\n Solution:");
		
		
		for (int k = 0; k < nDrivers; ++k) {
			LinkedList<Integer> passengerLists = new LinkedList<Integer>();
			for (int i = 0; i < nNodes; ++i) {
				for (int j = 0; j < nNodes; ++j) {
					if (travelFromTo[i][j][k].get(GRB.DoubleAttr.X) == 1) {
						System.out.println("Driver " +Integer.toString(k)+ " go from " +Integer.toString(i)+ " to "+Integer.toString(j));
						if (j < nPassengers) {
							passengerLists.add(j);
						}
					}
				}
			}
			System.out.println("\n Driver "+Integer.toString(k)+ " passengers:");
			System.out.println(passengerLists);
			System.out.println("\n");
		}
		
		System.out.println("\n");
		return;
	}
}

