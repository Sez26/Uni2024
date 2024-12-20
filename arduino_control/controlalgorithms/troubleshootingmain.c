  // // Store errors after 5 seconds, up to 5000 samples
  // if (running_time >= 10 && error_index < num_samples && !errors_collected) {
  //   error1_array[error_index] = output1;
  //   error2_array[error_index] = output2;
  //   error_index++;

  //   // After collecting 5000 samples, calculate min, max, and average
  //   if (error_index == num_samples) {
  //     errors_collected = true;

  //     // Initialize min and max values
  //     min_error1 = error1_array[0];
  //     max_error1 = error1_array[0];
  //     min_error2 = error2_array[0];
  //     max_error2 = error2_array[0];

  //     // Calculate min, max, and sum
  //     for (int i = 0; i < num_samples; i++) {
  //       // For error1
  //       if (error1_array[i] < min_error1) min_error1 = error1_array[i];
  //       if (error1_array[i] > max_error1) max_error1 = error1_array[i];
  //       sum_error1 += abs(error1_array[i]);

  //       // For error2
  //       if (error2_array[i] < min_error2) min_error2 = error2_array[i];
  //       if (error2_array[i] > max_error2) max_error2 = error2_array[i];
  //       sum_error2 += abs(error2_array[i]);
  //     }
  //         // Calculate average
  //     double avg_error1 = sum_error1 / num_samples;
  //     double avg_error2 = sum_error2 / num_samples;

  //     // Print the results
  //     Serial.println("Collected 5000 error samples:");
  //     Serial.print("Error1 - Min: "); Serial.print(min_error1);
  //     Serial.print(", Max: "); Serial.print(max_error1);
  //     Serial.print(", Average: "); Serial.println(avg_error1);

  //     Serial.print("Error2 - Min: "); Serial.print(min_error2);
  //     Serial.print(", Max: "); Serial.print(max_error2);
  //     Serial.print(", Average: "); Serial.println(avg_error2);

  //     delay(10000);
  //   }
  // }

  // Print target and position to see the response every print_interval times around the loop
  //interval_count = interval_count + 1;
  //if (interval_count >= print_interval) {
    //Serial.print("ref index:"); Serial.print(ref_index); Serial.println();

    // //old printing
    // Serial.print("motor1 ref; "); Serial.print(th_1[ref_index]);Serial.print(";");
    // Serial.print("motor2 ref; "); Serial.print(th_2[ref_index]);Serial.print(";");
    // Serial.print("Time;"); Serial.print(running_time, 4); Serial.print(";");
    // Serial.print("encoder1;"); Serial.print(encoder_count_volatile_motor1); Serial.print(";");
    // Serial.print("encoder2;"); Serial.print(encoder_count_volatile_motor2);Serial.print(";");
    // //Serial.print("Target_counts_1;"); Serial.print(target_counts_1); Serial.print(";");
    // Serial.print("Error1;"); Serial.print(output1); Serial.print(";");
    // //Serial.print("Target_counts_2;"); Serial.print(target_counts_2); Serial.print(";");
    // Serial.print("Error2;"); Serial.print(output2); Serial.print(";");Serial.println();

    // better formatting printing
    // Serial.print("motor1 ref; "); Serial.print(";");
    // Serial.print("motor2 ref; "); Serial.print(";");
    // Serial.print("Time;"); Serial.print(";");
    // Serial.print("encoder1;"); Serial.print(";");
    // Serial.print("encoder2;"); Serial.print(";");
    // Serial.print("Error1;"); Serial.print(";");
    // Serial.print("Error2;"); Serial.print(";");
    Serial.print("Time;"); Serial.print(";");
    Serial.print(running_time, 4); Serial.print(";");
    // Serial.print(th_1[ref_index]);Serial.print(";");
    // Serial.print(th_2[ref_index] + calibration_pos2);Serial.print(";");
    Serial.print("encoder1;"); Serial.print(";");
    Serial.print(encoder_count_volatile_motor1); Serial.print(";");
    Serial.print("encoder2;"); Serial.print(";");
    Serial.print(encoder_count_volatile_motor2);Serial.print(";");
    Serial.print("Error1;"); Serial.print(";");
    Serial.print(output1); Serial.print(";");
    Serial.print("Error2;"); Serial.print(";");
    Serial.print(output2); Serial.print(";");
    Serial.print(ref_index);
    Serial.println();
  //}
