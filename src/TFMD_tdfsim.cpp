/**********************************************************************/
/*  Parallel-Fault Event-Driven Transition Delay Fault Simulator      */
/*                                                                    */
/*           Author: Jun-Wei(Nick) Liang                              */
/*           last update : 05/31/2020                                 */
/**********************************************************************/

#include "atpg.h"

/* pack 16 faults into one packet.  simulate 16 faults togeter. 
 * the following variable name is somewhat misleading */
#define num_of_pattern 16

void ATPG::NOPrint_transition_delay_fault_simulation(int &total_detect_num) {
  int i;
  int current_detect_num = 0;

  /* for every vector */
  for (i = vectors.size() - 1; i >= 0; i--) {
    tdfault_sim_a_vector(vectors[i], current_detect_num);
    total_detect_num += current_detect_num;
    //fprintf(...);
  }
}// fault_simulate_vectors
//======================================================================================================//
//To modify the tdfsim.cpp's function
// 1. I change flist_undetct into TFMD
// 2. I change "f.detect = true" into "f.detected_time++"
// 3. fault dropping happens if f.detected_time >= detected_num
void ATPG::TFMD_transition_delay_fault_simulation(int &total_detect_num, 
	int &last_round_vectors_head, forward_list<fptr> & TFMD_container) {
  int i;
  int current_detect_num = 0;

  /* for every vector */
  for (i = vectors.size() - 1; i >= last_round_vectors_head; i--) {
    TFMD_tdfault_sim_a_vector(vectors[i], current_detect_num, TFMD_container);
    total_detect_num += current_detect_num;
    //fprintf(...);
  }
}// fault_simulate_vectors

void ATPG::TFMD_tdfault_sim_a_vector(const string &vec, int &num_of_current_detect,
	forward_list<fptr> & TFMD_container) {
		
  int i, nckt;
  fptr f;

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

  sim(); /* do a fault-free simulation, see sim.c */
  for (auto pos = TFMD_container.cbegin(); pos != TFMD_container.cend(); ++pos) {
    f = *pos;
    if (f->fault_type == sort_wlist[f->to_swlist]->value) {
      f->activate = TRUE;
    } else
      f->activate = FALSE;
  }

  TFMD_tdfault_sim_a_vector2(vec, num_of_current_detect, TFMD_container);

}

/* fault simulate a single test vector */
void ATPG::TFMD_tdfault_sim_a_vector2(const string &vec, int &num_of_current_detect,
	forward_list<fptr> & TFMD_container) {
		
  wptr w, faulty_wire;
  /* array of 16 fptrs, which points to the 16 faults in a simulation packet  */
  fptr simulated_fault_list[num_of_pattern];
  fptr f;
  int fault_type;
  int i, start_wire_index, nckt;
  int num_of_fault;

  num_of_fault = 0; // counts the number of faults in a packet

  /* num_of_current_detect is used to keep track of the number of undetected
   * faults detected by this vector, initialize it to zero */
  num_of_current_detect = 0;

  /* Keep track of the minimum wire index of 16 faults in a packet.
   * the start_wire_index is used to keep track of the
   * first gate that needs to be evaluated.
   * This reduces unnecessary check of scheduled events.*/
  start_wire_index = 10000;

  /* for every input, set its value to the current vector value */
  for (i = 0; i < cktin.size(); i++) {
    if (i == 0)
      cktin[i]->value = ctoi(vec[cktin.size()]);
    else
      cktin[i]->value = ctoi(vec[i - 1]);
  }

  /* initialize the circuit - mark all inputs as changed and all other
   * nodes as unknown (2) */
  nckt = sort_wlist.size();
  for (i = 0; i < nckt; i++) {
    if (i < cktin.size()) {
      sort_wlist[i]->set_changed();
    } else {
      sort_wlist[i]->value = U;
    }
  }

  sim(); /* do a fault-free simulation, see sim.c */
  if (debug) { display_io(); }

  /* expand the fault-free 0,1,2 value into 32 bits (2 = unknown)
   * and store it in wire_value1 (good value) and wire_value2 (faulty value)*/
  for (i = 0; i < nckt; i++) {
    switch (sort_wlist[i]->value) {
      case 1:
        sort_wlist[i]->wire_value1 = ALL_ONE;  // 11 represents logic one
        sort_wlist[i]->wire_value2 = ALL_ONE;
        break;
      case 2:
        sort_wlist[i]->wire_value1 = 0x55555555; // 01 represents unknown
        sort_wlist[i]->wire_value2 = 0x55555555;
        break;
      case 0:
        sort_wlist[i]->wire_value1 = ALL_ZERO; // 00 represents logic zero
        sort_wlist[i]->wire_value2 = ALL_ZERO;
        break;
    }
  } // for i

  /* walk through every undetected fault
   * the undetected fault list is linked by pnext_undetect */
  for (auto pos = TFMD_container.cbegin(); pos != TFMD_container.cend(); ++pos) {
    int fault_detected[num_of_pattern] = {0};
    f = *pos;
    if (f->detect == REDUNDANT) { continue; } /* ignore redundant faults */
    if (f->activate == FALSE) {
      if ((next(pos, 1) == TFMD_container.cend()) && num_of_fault > 0) {
        goto do_fsim;
      } else { continue; }
    } /* ignore redundant faults */

    /* consider only active (aka. excited) fault
     * (sa1 with correct output of 0 or sa0 with correct output of 1) */
    if (f->fault_type != sort_wlist[f->to_swlist]->value) {

      /* if f is a primary output or is directly connected to an primary output
       * the fault is detected */
      if ((f->node->type == OUTPUT) ||
          (f->io == GO && sort_wlist[f->to_swlist]->is_output())) {
        if(f->detected_time < detected_num) f->detected_time++;
      } else {

        /* if f is an gate output fault */
        if (f->io == GO) {

          /* if this wire is not yet marked as faulty, mark the wire as faulty
           * and insert the corresponding wire to the list of faulty wires. */
          if (!(sort_wlist[f->to_swlist]->is_faulty())) {
            sort_wlist[f->to_swlist]->set_faulty();
            wlist_faulty.push_front(sort_wlist[f->to_swlist]);
          }

          /* add the fault to the simulated fault list and inject the fault */
          simulated_fault_list[num_of_fault] = f;
          inject_fault_value(sort_wlist[f->to_swlist], num_of_fault, f->fault_type);

          /* mark the wire as having a fault injected
           * and schedule the outputs of this gate */
          sort_wlist[f->to_swlist]->set_fault_injected();
          for (auto pos_n = sort_wlist[f->to_swlist]->onode.cbegin(),
                   end_n = sort_wlist[f->to_swlist]->onode.cend(); pos_n != end_n; ++pos_n) {
            (*pos_n)->owire.front()->set_scheduled();
          }

          /* increment the number of simulated faults in this packet */
          num_of_fault++;
          /* start_wire_index keeps track of the smallest level of fault in this packet.
           * this saves simulation time.  */
          start_wire_index = min(start_wire_index, f->to_swlist);
        }  // if gate output fault

          /* the fault is a gate input fault */
        else {

          /* if the fault is propagated, set faulty_wire equal to the faulty wire.
           * faulty_wire is the gate output of f.  */
          faulty_wire = get_faulty_wire(f, fault_type);
          if (faulty_wire != nullptr) {

            /* if the faulty_wire is a primary output, it is detected */
            if (faulty_wire->is_output()) {
              if(f->detected_time < detected_num) f->detected_time++;
            } else {
              /* if faulty_wire is not already marked as faulty, mark it as faulty
               * and add the wire to the list of faulty wires. */
              if (!(faulty_wire->is_faulty())) {
                faulty_wire->set_faulty();
                wlist_faulty.push_front(faulty_wire);
              }

              /* add the fault to the simulated list and inject it */
              simulated_fault_list[num_of_fault] = f;
              inject_fault_value(faulty_wire, num_of_fault, fault_type);

              /* mark the faulty_wire as having a fault injected
               *  and schedule the outputs of this gate */
              faulty_wire->set_fault_injected();
              for (auto pos_n = faulty_wire->onode.cbegin(), end_n = faulty_wire->onode.cend();
                   pos_n != end_n; ++pos_n) {
                (*pos_n)->owire.front()->set_scheduled();
              }

              num_of_fault++;
              start_wire_index = min(start_wire_index, f->to_swlist);
            }
          }
        }
      } // if  gate input fault
    } // if fault is active


    /*
     * fault simulation of a packet
     */

    /* if this packet is full (16 faults)
     * or there is no more undetected faults remaining (pos points to the final element of flist_undetect),
     * do the fault simulation */
    if ((num_of_fault == num_of_pattern) || (next(pos, 1) == TFMD_container.cend())) {
      do_fsim:
      /* starting with start_wire_index, evaulate all scheduled wires
       * start_wire_index helps to save time. */
      for (i = start_wire_index; i < nckt; i++) {
        if (sort_wlist[i]->is_scheduled()) {
          sort_wlist[i]->remove_scheduled();
          fault_sim_evaluate(sort_wlist[i]);
        }
      } /* event evaluations end here */

      /* pop out all faulty wires from the wlist_faulty
      * if PO's value is different from good PO's value, and it is not unknown
      * then the fault is detected.
      *
      * IMPORTANT! remember to reset the wires' faulty values back to fault-free values.
      */
      while (!wlist_faulty.empty()) {
        w = wlist_faulty.front();
        wlist_faulty.pop_front();
        w->remove_faulty();
        w->remove_fault_injected();
        w->set_fault_free();
        /*TODO*/
        //Hint:Use mask to get the value of faulty wire and check every fault in packet
        if (w->is_output()) { // if primary output
          for (i = 0; i < num_of_fault; i++) { // check every undetected fault
            if (simulated_fault_list[i]->detected_time < detected_num) {
              if ((w->wire_value2 & Mask[i]) ^    // if value1 != value2
                  (w->wire_value1 & Mask[i])) {
                if (((w->wire_value2 & Mask[i]) ^ Unknown[i]) &&  // and not unknowns
                    ((w->wire_value1 & Mask[i]) ^ Unknown[i])) {
                  fault_detected[i] = 1;// then the fault is detected
                }
              }
            }
          }
        }
        w->wire_value2 = w->wire_value1;  // reset to fault-free values
        /*TODO*/
      } // pop out all faulty wires
      for (i = 0; i < num_of_fault; i++) {
        if (fault_detected[i] == 1) {
          if(simulated_fault_list[i]->detected_time < detected_num) 
		      simulated_fault_list[i]->detected_time++;
        }
      }
      num_of_fault = 0;  // reset the counter of faults in a packet
      start_wire_index = 10000;  //reset this index to a very large value.
    } // end fault sim of a packet
  } // end loop. for f = flist

  /* fault dropping  */
  TFMD_container.remove_if(
      [&](const fptr fptr_ele) {
        if (fptr_ele->detected_time >= detected_num) {
          string IO;
          /*if(fptr_ele->io == GO) IO = "GO";
          else IO = "GI";
          if(fptr_ele->fault_type == STR)
            cout << "fault "<<  fptr_ele->fault_no<< ": STR at wire-"<< sort_wlist[fptr_ele->to_swlist]->name<< ", "<< IO<< " of "<< fptr_ele->node->name <<endl;
          else
            cout << "fault "<<  fptr_ele->fault_no<< ": STF at wire- "<< sort_wlist[fptr_ele->to_swlist]->name<< ", "<< IO<< " of "<< fptr_ele->node->name <<endl;*/
          num_of_current_detect += fptr_ele->eqv_fault_num;
          return true;
        } else {
          return false;
        }
      });

}/* end of fault_sim_a_vector */
