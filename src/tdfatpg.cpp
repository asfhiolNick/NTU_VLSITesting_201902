/**********************************************************************/
/*           Constructors of the classes defined in atpg.h            */
/*           ATPG top-level functions                                 */
/*           Author: Jun-Wei (Nick) Liang                             */
/*           last update : 05/30/2020                                 */
/**********************************************************************/

#include "atpg.h"
#include "assert.h"
//2020-week13 modify
//Read-in V1, simulate V1 on ckt, then check f is activated or not. 
bool ATPG::tdf_sim_vector(const string &vec, fptr f) {
  int i, nckt;

  for (i = 0; i < cktin.size(); i++) {
    cktin[i]->value = ctoi(vec[i]);
  }

  nckt = sort_wlist.size();
  for (i = 0; i < nckt; i++) {
    if (i < cktin.size()) {
      sort_wlist[i]->set_changed();
    } else {
      sort_wlist[i]->value = U;
    }
  }

  sim();
  
  if(f->fault_type == sort_wlist[f->to_swlist]->value) return TRUE;
  else                                                 return FALSE;
}
//end of tdf_sim_vector
//


//2020-week13 modify
//For all faults, run single-trans-delay-fault-ATPG to generate T1.
//If this is 1-det case, then we finished.
//Else, this is N-det case, then we continued it.
//We need to do "TFMD_tdfsim" and "Fk_atpg" for N-1 times.
void ATPG::tdftest() {
    string vec;
    //2020-week13 modify
    string vec0, vec1;
    //
    int current_detect_num = 0;
    int total_detect_num = 0;
    int total_no_of_backtracks = 0;  // accumulative number of backtracks
    int current_backtracks = 0;
    int no_of_aborted_faults = 0;
    int no_of_redundant_faults = 0;
    int no_of_calls = 0;

    fptr fault_under_test = flist_undetect.front();

    /* test generation mode */
    /* Figure 5 in the PODEM paper */
    while (fault_under_test != nullptr) {
        switch (podem(fault_under_test, current_backtracks)) {
            case TRUE:
                /* form a vector */
                vec.clear();
                for (wptr w: cktin) {
                    vec.push_back(itoc(w->value));
                }
                //2020-week13 modify
                vec0.clear(), vec1.clear();
                vec0 = vec.substr(1, vec.size()-1)+"0";
                vec1 = vec.substr(1, vec.size()-1)+"1";
                
                if( tdf_sim_vector(vec0, fault_under_test) ){
                	cout << "T'" << vec0 << " " << vec[0] << "'" << endl;
                	vectors.push_back((vec0+vec[0]));
                	
                	/*by defect, we want only one pattern per fault */
                	/*run a fault simulation, drop ALL detected faults */
                	if (total_attempt_num == 1) {
                    	tdfault_sim_a_vector(vec0+vec[0], current_detect_num);
                    	total_detect_num += current_detect_num;
                	}
                    	/* If we want mutiple petterns per fault,
                     	* NO fault simulation.  drop ONLY the fault under test */
                	else {
                    	fault_under_test->detect = TRUE;
                    	/* drop fault_under_test */
                    	flist_undetect.remove(fault_under_test);
                	}
                	in_vector_no++;
                } else if( tdf_sim_vector(vec1, fault_under_test) ){
                	cout << "T'" << vec1 << " " << vec[0] << "'" << endl;
                	vectors.push_back((vec1+vec[0]));

                	/*by defect, we want only one pattern per fault */
                	/*run a fault simulation, drop ALL detected faults */
                	if (total_attempt_num == 1) {
                    	tdfault_sim_a_vector(vec1+vec[0], current_detect_num);
                    	total_detect_num += current_detect_num;
                	}
                    	/* If we want mutiple petterns per fault,
                     	* NO fault simulation.  drop ONLY the fault under test */
                	else {
                    	fault_under_test->detect = TRUE;
                    	/* drop fault_under_test */
                    	flist_undetect.remove(fault_under_test);
                	}
                	in_vector_no++;
                } else{
                    no_of_aborted_faults++;
                }
                //
                break;
            case FALSE:fault_under_test->detect = REDUNDANT;
                no_of_redundant_faults++;
                break;

            case MAYBE:no_of_aborted_faults++;
                break;
        }
        fault_under_test->test_tried = true;
        fault_under_test = nullptr;
        for (fptr fptr_ele: flist_undetect) {
            if (!fptr_ele->test_tried) {
                fault_under_test = fptr_ele;
                break;
            }
        }
        total_no_of_backtracks += current_backtracks; // accumulate number of backtracks
        no_of_calls++;
    }
 
//tdfsim's work: clean data structure and run tdf-simulation
    flist_undetect.clear();
    for (auto pos = flist.cbegin(); pos != flist.cend(); ++pos) {
    	fptr f = (*pos).get();
    	f->activate = false;
    	f->detect   = false;
    	f->test_tried = false;
    	f->detected_time = 0;
    	flist_undetect.push_front(f);
    }
    total_detect_num = 0;
    
    if(detected_num == 1) transition_delay_fault_simulation(total_detect_num);
    else                  NOPrint_transition_delay_fault_simulation(total_detect_num);
//We have stored T1 into vectors[], now we clean the data structure and collect TF_MD 
    if(detected_num > 1){
    	TFMD.clear();
    	for (auto pos = flist.cbegin(); pos != flist.cend(); ++pos) {
    		fptr f = (*pos).get();
    		if(f->detect == true) TFMD.push_front(f); 
    		f->activate = false;
    		f->detect   = false;
    		f->test_tried = false;
    		f->detected_time = 0;
    	}    
    	total_detect_num = 0;
        
        int last_round_vectors_head = 0;
    	for(int k=1; k<detected_num; ++k){
    		cout << "###k = " << k << " ###"<< endl;
    		//Perform multiple-fault-simulation on TFMD faults with test pattern, T_k
			//T_k:=vectors[last_round_vectors_head ~ vectors.size()-1]
    		TFMD_transition_delay_fault_simulation(total_detect_num, last_round_vectors_head, TFMD);
    		last_round_vectors_head = vectors.size(); 
    		
    		//Collect F_k, using container "flist_undetect"
    		flist_undetect.clear();
    		vector<FAULT> copy = {};
    		for(auto pos = TFMD.cbegin(); pos != TFMD.cend(); ++pos){
    			if((*pos)->detected_time == k){
    				copy.push_back(**pos);
    				(*pos)->activate = false;
    				(*pos)->detect = false;
    				(*pos)->test_tried = false;
    				flist_undetect.push_front((*pos));
    			} 
    		}
    		flist_undetect.reverse();
    		//Do ATPG on Fk, generate test patterns T_(k+1)
    		Fk_atpg(flist_undetect);
    		//Return the situation before ATPG on Fk
    		int count = 0;
    		//Warning
    		//1. detected_time would not change
    		//2. Since I use detected_time to find back faults, so don't clear detected_time to 0, then restore it
    		for(auto pos = TFMD.cbegin(); pos != TFMD.cend(); ++pos){
    			if((*pos)->detected_time == k){
    				assert(count < copy.size());
    				FAULT f = copy[count];
					(*pos)->activate = f.activate;
					(*pos)->detect = f.detect;
					(*pos)->test_tried = f.test_tried;  
					count ++;
    			} 
    		}
    		flist_undetect.clear();
    	}
    	
    	//Finally, run the single simulation on all faults
    	flist_undetect.clear();
    	for (auto pos = flist.cbegin(); pos != flist.cend(); ++pos) {
    		fptr f = (*pos).get();
    		f->activate = false;
   		 	f->detect   = false;
    		f->test_tried = false;
    		f->detected_time = 0;
    		flist_undetect.push_front(f);
    	}
    	total_detect_num = 0;
    	//single-detected fsim -> transition_delay_fault_simulation(total_detect_num)
    	//multiply-detected fsim -> TFMD_transition_delay_fault_simulation(total_detect_num, starting, flist_undetect)
    	int starting = 0;
		TFMD_transition_delay_fault_simulation(total_detect_num, starting, flist_undetect);    	
	}

    display_undetect();
    /*fprintf(stdout, "\n");
    fprintf(stdout, "#number of aborted faults = %d\n", no_of_aborted_faults);
    fprintf(stdout, "\n");
    fprintf(stdout, "#number of redundant faults = %d\n", no_of_redundant_faults);
    fprintf(stdout, "\n");
    fprintf(stdout, "#number of calling podem1 = %d\n", no_of_calls);
    fprintf(stdout, "\n");
    fprintf(stdout, "#total number of backtracks = %d\n", total_no_of_backtracks);*/
    printf("\n# Result:\n");
    printf("-----------------------\n");
    cout << "# Number of test: " << vectors.size() << " [New item]" << endl;
    printf("# total transition delay faults: %d\n", num_of_tdf_fault);
    printf("# total detected faults: %d\n", total_detect_num);
    printf("# fault coverage: %lf %\n", (double) total_detect_num / (double) num_of_tdf_fault * 100);
    return;    
}
//end of tdftest
//


//2020-week13 modify
//Given a copied fault-list, Fk, we run the single-tdf-ATPG to generate T_(k+1)
//1. change flist_undetect->Fk
void ATPG::Fk_atpg(forward_list<fptr>& Fk) {
    string vec;
    //2020-week13 modify
    string vec0, vec1;
    //
    int current_detect_num = 0;
    int total_detect_num = 0;
    int total_no_of_backtracks = 0;  // accumulative number of backtracks
    int current_backtracks = 0;
    int no_of_aborted_faults = 0;
    int no_of_redundant_faults = 0;
    int no_of_calls = 0;

    fptr fault_under_test = Fk.front();

    /* test generation mode */
    /* Figure 5 in the PODEM paper */
    while (fault_under_test != nullptr) {
        switch (podem(fault_under_test, current_backtracks)) {
            case TRUE:
                /* form a vector */
                vec.clear();
                for (wptr w: cktin) {
                    vec.push_back(itoc(w->value));
                }
                //2020-week13 modify
                vec0.clear(), vec1.clear();
                vec0 = vec.substr(1, vec.size()-1)+"0";
                vec1 = vec.substr(1, vec.size()-1)+"1";
                
                if( tdf_sim_vector(vec0, fault_under_test) ){
                	cout << "T'" << vec0 << " " << vec[0] << "'" << endl;
                	vectors.push_back((vec0+vec[0]));
                	
                	/*by defect, we want only one pattern per fault */
                	/*run a fault simulation, drop ALL detected faults */
                	if (total_attempt_num == 1) {
                    	tdfault_sim_a_vector(vec0+vec[0], current_detect_num);
                    	total_detect_num += current_detect_num;
                	}
                    	/* If we want mutiple petterns per fault,
                     	* NO fault simulation.  drop ONLY the fault under test */
                	else {
                    	fault_under_test->detect = TRUE;
                    	/* drop fault_under_test */
                    	Fk.remove(fault_under_test);
                	}
                	in_vector_no++;
                } else if( tdf_sim_vector(vec1, fault_under_test) ){
                	cout << "T'" << vec1 << " " << vec[0] << "'" << endl;
                	vectors.push_back((vec1+vec[0]));

                	/*by defect, we want only one pattern per fault */
                	/*run a fault simulation, drop ALL detected faults */
                	if (total_attempt_num == 1) {
                    	tdfault_sim_a_vector(vec1+vec[0], current_detect_num);
                    	total_detect_num += current_detect_num;
                	}
                    	/* If we want mutiple petterns per fault,
                     	* NO fault simulation.  drop ONLY the fault under test */
                	else {
                    	fault_under_test->detect = TRUE;
                    	/* drop fault_under_test */
                    	Fk.remove(fault_under_test);
                	}
                	in_vector_no++;
                } else{
                    no_of_aborted_faults++;
                }
                //
                break;
            case FALSE:fault_under_test->detect = REDUNDANT;
                no_of_redundant_faults++;
                break;

            case MAYBE:no_of_aborted_faults++;
                break;
        }
        fault_under_test->test_tried = true;
        fault_under_test = nullptr;
        for (fptr fptr_ele: Fk) {
            if (!fptr_ele->test_tried) {
                fault_under_test = fptr_ele;
                break;
            }
        }
        total_no_of_backtracks += current_backtracks; // accumulate number of backtracks
        no_of_calls++;
    }
	return;
} 
//end of Fk_atpg
//
