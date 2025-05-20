
#include <Arduino.h>
#include <math.h>


// Activity type mapping:
// 0: downSit
// 1: freeFall
// 2: runFall
// 3: runSit
// 4: walkFall
// 5: walkSit


void extract_features(float accel_x[], float accel_y[], float accel_z[], 
                     int window_size, float features[]) {
  float mean_x = 0, mean_y = 0, mean_z = 0;
  float min_x = accel_x[0], min_y = accel_y[0], min_z = accel_z[0];
  float max_x = accel_x[0], max_y = accel_y[0], max_z = accel_z[0];
  
  for (int i = 0; i < window_size; i++) {
    mean_x += accel_x[i];
    mean_y += accel_y[i];
    mean_z += accel_z[i];
    
    if (accel_x[i] < min_x) min_x = accel_x[i];
    if (accel_y[i] < min_y) min_y = accel_y[i];
    if (accel_z[i] < min_z) min_z = accel_z[i];
    
    if (accel_x[i] > max_x) max_x = accel_x[i];
    if (accel_y[i] > max_y) max_y = accel_y[i];
    if (accel_z[i] > max_z) max_z = accel_z[i];
  }
  
  mean_x /= window_size;
  mean_y /= window_size;
  mean_z /= window_size;
  
  float var_x = 0, var_y = 0, var_z = 0;
  for (int i = 0; i < window_size; i++) {
    var_x += (accel_x[i] - mean_x) * (accel_x[i] - mean_x);
    var_y += (accel_y[i] - mean_y) * (accel_y[i] - mean_y);
    var_z += (accel_z[i] - mean_z) * (accel_z[i] - mean_z);
  }
  
  var_x /= window_size;
  var_y /= window_size;
  var_z /= window_size;
  
  float std_x = sqrt(var_x);
  float std_y = sqrt(var_y);
  float std_z = sqrt(var_z);
  
  float mag_values[window_size];
  float mean_mag = 0, min_mag = 0, max_mag = 0;
  
  for (int i = 0; i < window_size; i++) {
    mag_values[i] = sqrt(accel_x[i]*accel_x[i] + accel_y[i]*accel_y[i] + accel_z[i]*accel_z[i]);
    mean_mag += mag_values[i];
    
    if (i == 0 || mag_values[i] < min_mag) min_mag = mag_values[i];
    if (i == 0 || mag_values[i] > max_mag) max_mag = mag_values[i];
  }
  
  mean_mag /= window_size;
  
  int idx = 0;
  features[idx++] = mean_x;
  features[idx++] = std_x;
  features[idx++] = min_x;
  features[idx++] = max_x;
  features[idx++] = max_x - min_x;
  
  features[idx++] = mean_y;
  features[idx++] = std_y;
  features[idx++] = min_y;
  features[idx++] = max_y;
  features[idx++] = max_y - min_y;
  
  features[idx++] = mean_z;
  features[idx++] = std_z;
  features[idx++] = min_z;
  features[idx++] = max_z;
  features[idx++] = max_z - min_z;
  
  features[idx++] = mean_mag;
  features[idx++] = max_mag;
  features[idx++] = max_mag - min_mag;
}


int tree_0(float features[]) {
if (features[4] <= -0.068496f) {
  if (features[84] <= 0.128901f) {
    if (features[16] <= 0.306931f) {
      if (features[58] <= -0.385997f) {
        return 4;
      } else {
        if (features[64] <= -0.067044f) {
          return 4;
        } else {
          if (features[28] <= -0.358184f) {
            return 1;
          } else {
            if (features[20] <= 0.289083f) {
              return 5;
            } else {
              return 4;
            }
          }
        }
      }
    } else {
      return 4;
    }
  } else {
    if (features[33] <= -1.479882f) {
      return 1;
    } else {
      return 4;
    }
  }
} else {
  if (features[2] <= 0.497363f) {
    if (features[26] <= 0.130526f) {
      if (features[19] <= -0.522012f) {
        return 4;
      } else {
        if (features[19] <= -0.436631f) {
          return 1;
        } else {
          return 0;
        }
      }
    } else {
      if (features[51] <= -0.660711f) {
        return 2;
      } else {
        return 0;
      }
    }
  } else {
    if (features[83] <= -0.636852f) {
      return 4;
    } else {
      return 1;
    }
  }
}
}

int tree_1(float features[]) {
if (features[31] <= 0.086712f) {
  if (features[5] <= -0.549491f) {
    if (features[92] <= -0.428181f) {
      return 1;
    } else {
      return 2;
    }
  } else {
    if (features[34] <= 0.715084f) {
      if (features[45] <= -0.472564f) {
        if (features[2] <= -0.043532f) {
          if (features[62] <= -0.338661f) {
            return 2;
          } else {
            if (features[86] <= -0.001097f) {
              return 2;
            } else {
              if (features[23] <= 0.586740f) {
                return 1;
              } else {
                if (features[22] <= -0.366980f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              }
            }
          }
        } else {
          if (features[29] <= -0.496700f) {
            return 4;
          } else {
            if (features[66] <= -0.308580f) {
              if (features[67] <= -0.074126f) {
                return 4;
              } else {
                return 1;
              }
            } else {
              if (features[58] <= -0.521695f) {
                if (features[21] <= -0.433023f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                if (features[39] <= -0.285961f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              }
            }
          }
        }
      } else {
        return 0;
      }
    } else {
      if (features[42] <= -0.426508f) {
        if (features[66] <= -0.273484f) {
          return 4;
        } else {
          return 0;
        }
      } else {
        return 0;
      }
    }
  }
} else {
  if (features[8] <= -0.408175f) {
    if (features[28] <= 0.143096f) {
      if (features[1] <= -0.457904f) {
        return 4;
      } else {
        if (features[78] <= 0.011519f) {
          if (features[47] <= -0.236300f) {
            if (features[24] <= -0.352803f) {
              if (features[23] <= -0.003422f) {
                if (features[63] <= -0.047573f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                return 1;
              }
            } else {
              if (features[12] <= 0.131612f) {
                if (features[53] <= -0.238338f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                if (features[0] <= 0.702192f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              }
            }
          } else {
            if (features[41] <= -0.545306f) {
              if (features[74] <= -0.169132f) {
                return 4;
              } else {
                return 1;
              }
            } else {
              return 4;
            }
          }
        } else {
          if (features[43] <= -0.604245f) {
            return 4;
          } else {
            return 0;
          }
        }
      }
    } else {
      if (features[32] <= -0.322861f) {
        if (features[23] <= 0.548078f) {
          return 0;
        } else {
          return 0;
        }
      } else {
        if (features[4] <= -0.590470f) {
          return 4;
        } else {
          if (features[81] <= 0.315009f) {
            return 0;
          } else {
            return 4;
          }
        }
      }
    }
  } else {
    if (features[36] <= -0.377099f) {
      return 0;
    } else {
      return 0;
    }
  }
}
}

int tree_2(float features[]) {
if (features[6] <= -0.668010f) {
  if (features[69] <= -0.310184f) {
    if (features[93] <= -0.385861f) {
      return 0;
    } else {
      return 4;
    }
  } else {
    if (features[69] <= -0.316297f) {
      if (features[88] <= -0.963740f) {
        if (features[10] <= -0.339724f) {
          if (features[0] <= -0.969962f) {
            if (features[21] <= 0.196460f) {
              return 4;
            } else {
              return 2;
            }
          } else {
            if (features[2] <= 0.468488f) {
              if (features[20] <= 0.474655f) {
                if (features[13] <= -0.184366f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                return 4;
              }
            } else {
              if (features[51] <= -0.785470f) {
                if (features[59] <= -0.739574f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                return 2;
              }
            }
          }
        } else {
          return 4;
        }
      } else {
        if (features[13] <= -0.250247f) {
          if (features[91] <= -0.346015f) {
            if (features[81] <= -0.835718f) {
              return 4;
            } else {
              return 5;
            }
          } else {
            if (features[31] <= -1.089468f) {
              return 4;
            } else {
              if (features[88] <= -0.429785f) {
                if (features[0] <= 0.561558f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                if (features[85] <= -1.176969f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              }
            }
          }
        } else {
          if (features[37] <= -0.262431f) {
            return 5;
          } else {
            if (features[16] <= 0.237468f) {
              if (features[78] <= -1.008915f) {
                return 4;
              } else {
                if (features[35] <= 1.720182f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              }
            } else {
              return 4;
            }
          }
        }
      }
    } else {
      if (features[58] <= -0.486841f) {
        return 5;
      } else {
        return 4;
      }
    }
  }
} else {
  if (features[84] <= 0.252330f) {
    if (features[37] <= -0.430849f) {
      return 5;
    } else {
      return 0;
    }
  } else {
    if (features[16] <= 0.611376f) {
      return 0;
    } else {
      return 4;
    }
  }
}
}

int tree_3(float features[]) {
if (features[1] <= -0.017257f) {
  if (features[14] <= 0.569034f) {
    if (features[48] <= -0.128763f) {
      if (features[2] <= 0.456484f) {
        return 4;
      } else {
        return 5;
      }
    } else {
      if (features[57] <= -0.329482f) {
        if (features[76] <= 1.019405f) {
          if (features[53] <= -0.335452f) {
            return 5;
          } else {
            return 4;
          }
        } else {
          if (features[33] <= -1.233058f) {
            if (features[85] <= -0.616499f) {
              return 1;
            } else {
              return 4;
            }
          } else {
            if (features[30] <= -0.134196f) {
              if (features[21] <= 0.207515f) {
                if (features[4] <= -0.345969f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                if (features[33] <= -0.042439f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              }
            } else {
              return 0;
            }
          }
        }
      } else {
        return 5;
      }
    }
  } else {
    if (features[11] <= -0.475098f) {
      if (features[68] <= -0.172883f) {
        return 0;
      } else {
        if (features[16] <= -0.678849f) {
          if (features[64] <= 0.168564f) {
            return 2;
          } else {
            return 4;
          }
        } else {
          return 5;
        }
      }
    } else {
      return 2;
    }
  }
} else {
  if (features[39] <= 0.280460f) {
    if (features[2] <= 0.691349f) {
      return 1;
    } else {
      return 5;
    }
  } else {
    if (features[55] <= -0.249913f) {
      if (features[5] <= 0.495993f) {
        return 5;
      } else {
        if (features[33] <= -0.965892f) {
          return 2;
        } else {
          return 0;
        }
      }
    } else {
      if (features[44] <= -0.007642f) {
        if (features[2] <= 0.209309f) {
          if (features[43] <= -0.077447f) {
            return 4;
          } else {
            return 3;
          }
        } else {
          if (features[84] <= 0.742355f) {
            if (features[20] <= -0.452880f) {
              if (features[91] <= -0.368538f) {
                if (features[40] <= 0.578357f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                return 3;
              }
            } else {
              if (features[30] <= 0.698059f) {
                if (features[29] <= 1.198761f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                return 3;
              }
            }
          } else {
            return 1;
          }
        }
      } else {
        return 2;
      }
    }
  }
}
}

int tree_4(float features[]) {
if (features[6] <= -0.701096f) {
  if (features[91] <= -0.299964f) {
    if (features[21] <= 0.213395f) {
      return 5;
    } else {
      if (features[19] <= 0.702612f) {
        return 2;
      } else {
        if (features[89] <= -0.942667f) {
          return 0;
        } else {
          return 2;
        }
      }
    }
  } else {
    return 0;
  }
} else {
  if (features[3] <= -0.765808f) {
    if (features[36] <= -0.399061f) {
      if (features[67] <= -0.075524f) {
        return 2;
      } else {
        return 2;
      }
    } else {
      return 0;
    }
  } else {
    return 4;
  }
}
}


int detect_activity(float features[]) {
  int votes[6] = {0};
  int n_trees = 5;
  int i, max_votes = 0, predicted_class = 0;
  
  // Count votes for each class
  votes[tree_0(features)]++;
  votes[tree_1(features)]++;
  votes[tree_2(features)]++;
  votes[tree_3(features)]++;
  votes[tree_4(features)]++;
  
  // Find class with most votes
  for (i = 0; i < 6; i++) {
    if (votes[i] > max_votes) {
      max_votes = votes[i];
      predicted_class = i;
    }
  }
  
  return predicted_class;
}

bool detect_fall(float features[]) {
  // In multi-class setup, classes 0-2 might be falls (depends on encoding)
  // This needs to be adjusted based on your specific label encoding
  int activity = detect_activity(features);
  
  // Example: if classes 0, 1, 2 are fall activities (runFall, walkFall, freeFall)
  return (activity >= 0 && activity <= 2);
}

