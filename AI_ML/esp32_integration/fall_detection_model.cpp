
#include <Arduino.h>
#include <math.h>


// Activity type mapping:
// 0: downSit
// 1: freeFall
// 2: runFall
// 3: runSit
// 4: walkSit


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
if (features[4] <= -0.290791f) {
  if (features[84] <= -0.440984f) {
    if (features[49] <= -0.383311f) {
      return 4;
    } else {
      if (features[58] <= -0.520582f) {
        if (features[94] <= -0.355420f) {
          if (features[73] <= -0.258749f) {
            if (features[22] <= -0.520858f) {
              return 2;
            } else {
              if (features[10] <= 0.314283f) {
                if (features[93] <= -0.152328f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                return 0;
              }
            }
          } else {
            if (features[31] <= -0.428465f) {
              return 1;
            } else {
              return 2;
            }
          }
        } else {
          if (features[15] <= -0.653438f) {
            return 2;
          } else {
            return 0;
          }
        }
      } else {
        return 2;
      }
    }
  } else {
    if (features[11] <= -0.561817f) {
      return 1;
    } else {
      return 2;
    }
  }
} else {
  if (features[2] <= 0.707306f) {
    if (features[31] <= -1.146153f) {
      if (features[91] <= -0.536444f) {
        return 1;
      } else {
        return 1;
      }
    } else {
      return 0;
    }
  } else {
    if (features[88] <= 1.168189f) {
      if (features[51] <= 0.218257f) {
        return 0;
      } else {
        if (features[66] <= -0.246244f) {
          return 1;
        } else {
          return 4;
        }
      }
    } else {
      return 4;
    }
  }
}
}

int tree_1(float features[]) {
if (features[75] <= -0.068486f) {
  if (features[5] <= -0.423154f) {
    if (features[92] <= -0.412826f) {
      if (features[37] <= -0.203994f) {
        if (features[30] <= 0.583861f) {
          return 4;
        } else {
          if (features[65] <= -0.340614f) {
            return 0;
          } else {
            return 4;
          }
        }
      } else {
        if (features[2] <= 0.080118f) {
          return 4;
        } else {
          if (features[47] <= -0.341318f) {
            return 0;
          } else {
            return 4;
          }
        }
      }
    } else {
      return 0;
    }
  } else {
    if (features[34] <= 0.207870f) {
      if (features[63] <= -0.054724f) {
        if (features[77] <= 0.561045f) {
          if (features[93] <= -0.413110f) {
            return 4;
          } else {
            if (features[72] <= -0.412388f) {
              if (features[32] <= -0.013626f) {
                return 1;
              } else {
                return 2;
              }
            } else {
              return 4;
            }
          }
        } else {
          return 0;
        }
      } else {
        return 0;
      }
    } else {
      return 0;
    }
  }
} else {
  if (features[4] <= -0.547013f) {
    if (features[52] <= -0.352656f) {
      return 4;
    } else {
      if (features[19] <= 0.902453f) {
        if (features[65] <= -0.352913f) {
          if (features[42] <= -0.426100f) {
            return 4;
          } else {
            return 2;
          }
        } else {
          if (features[44] <= 0.628782f) {
            return 4;
          } else {
            return 2;
          }
        }
      } else {
        return 0;
      }
    }
  } else {
    return 2;
  }
}
}

int tree_2(float features[]) {
if (features[6] <= -0.468199f) {
  if (features[27] <= -0.453190f) {
    if (features[24] <= -0.295961f) {
      if (features[16] <= 0.651910f) {
        return 2;
      } else {
        if (features[63] <= -0.054637f) {
          if (features[39] <= -0.401280f) {
            if (features[87] <= 0.159174f) {
              return 0;
            } else {
              return 2;
            }
          } else {
            if (features[1] <= -0.615824f) {
              if (features[22] <= -0.386014f) {
                if (features[45] <= -0.396683f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                return 2;
              }
            } else {
              return 2;
            }
          }
        } else {
          if (features[27] <= -0.415847f) {
            return 4;
          } else {
            if (features[10] <= 0.084504f) {
              if (features[6] <= 0.682660f) {
                if (features[80] <= -0.623989f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                if (features[42] <= -0.533407f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              }
            } else {
              if (features[78] <= -0.157178f) {
                if (features[34] <= -0.643318f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                return 2;
              }
            }
          }
        }
      }
    } else {
      return 0;
    }
  } else {
    return 4;
  }
} else {
  if (features[84] <= 0.139037f) {
    if (features[62] <= -0.223026f) {
      return 2;
    } else {
      if (features[2] <= -0.018686f) {
        if (features[76] <= -1.176707f) {
          return 4;
        } else {
          return 2;
        }
      } else {
        if (features[47] <= 0.047828f) {
          if (features[76] <= -1.028933f) {
            if (features[66] <= -0.228448f) {
              return 1;
            } else {
              if (features[90] <= -0.447751f) {
                return 0;  // Safety check: Index out of bounds
              } else {
                return 0;  // Safety check: Index out of bounds
              }
            }
          } else {
            return 4;
          }
        } else {
          if (features[33] <= 0.729988f) {
            return 0;
          } else {
            if (features[27] <= -0.627478f) {
              if (features[69] <= -0.304752f) {
                return 0;  // Safety check: Index out of bounds
              } else {
                return 0;  // Safety check: Index out of bounds
              }
            } else {
              return 4;
            }
          }
        }
      }
    }
  } else {
    if (features[23] <= -0.202348f) {
      if (features[58] <= -0.529606f) {
        if (features[88] <= -0.202515f) {
          return 2;
        } else {
          return 0;
        }
      } else {
        return 0;
      }
    } else {
      return 4;
    }
  }
}
}

int tree_3(float features[]) {
if (features[1] <= -0.294208f) {
  if (features[14] <= 0.311884f) {
    if (features[8] <= -0.819151f) {
      return 2;
    } else {
      if (features[23] <= 0.672221f) {
        if (features[79] <= -0.561146f) {
          if (features[53] <= -0.376907f) {
            if (features[21] <= -0.443682f) {
              if (features[77] <= -1.207285f) {
                if (features[57] <= -0.242792f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                if (features[7] <= -0.606103f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              }
            } else {
              if (features[9] <= -0.565356f) {
                return 1;
              } else {
                if (features[48] <= -0.454617f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              }
            }
          } else {
            return 1;
          }
        } else {
          if (features[33] <= -1.167429f) {
            return 1;
          } else {
            if (features[45] <= -0.230467f) {
              return 0;
            } else {
              return 0;
            }
          }
        }
      } else {
        return 1;
      }
    }
  } else {
    if (features[80] <= -0.953636f) {
      return 4;
    } else {
      return 2;
    }
  }
} else {
  if (features[39] <= 0.222560f) {
    if (features[73] <= -0.274839f) {
      if (features[38] <= 1.331768f) {
        if (features[32] <= -0.338498f) {
          return 4;
        } else {
          if (features[50] <= -0.314809f) {
            if (features[47] <= -0.364351f) {
              return 4;
            } else {
              return 0;
            }
          } else {
            if (features[86] <= 0.048264f) {
              return 2;
            } else {
              if (features[61] <= -0.199014f) {
                return 0;  // Safety check: Index out of bounds
              } else {
                return 0;  // Safety check: Index out of bounds
              }
            }
          }
        }
      } else {
        if (features[7] <= -0.174082f) {
          if (features[25] <= -0.609396f) {
            if (features[89] <= -0.943326f) {
              return 0;
            } else {
              return 4;
            }
          } else {
            if (features[48] <= -0.123801f) {
              if (features[5] <= -0.177469f) {
                return 0;  // Safety check: Index out of bounds
              } else {
                return 0;  // Safety check: Index out of bounds
              }
            } else {
              if (features[53] <= -0.282784f) {
                return 0;  // Safety check: Index out of bounds
              } else {
                return 0;  // Safety check: Index out of bounds
              }
            }
          }
        } else {
          return 1;
        }
      }
    } else {
      if (features[52] <= 0.749916f) {
        if (features[3] <= -0.398608f) {
          return 4;
        } else {
          if (features[37] <= -0.157182f) {
            return 0;
          } else {
            return 1;
          }
        }
      } else {
        if (features[56] <= -0.807548f) {
          return 4;
        } else {
          return 0;
        }
      }
    }
  } else {
    if (features[51] <= 2.310232f) {
      return 1;
    } else {
      return 4;
    }
  }
}
}

int tree_4(float features[]) {
if (features[32] <= -0.348183f) {
  if (features[3] <= -0.594547f) {
    if (features[13] <= -0.519473f) {
      if (features[53] <= -0.335993f) {
        return 4;
      } else {
        if (features[61] <= -0.310613f) {
          if (features[9] <= -0.418337f) {
            return 0;
          } else {
            return 4;
          }
        } else {
          if (features[39] <= -0.409287f) {
            return 4;
          } else {
            if (features[47] <= -0.332508f) {
              if (features[92] <= -0.023377f) {
                if (features[13] <= 0.410659f) {
                  return 0;  // Safety check: Index out of bounds
                } else {
                  return 0;  // Safety check: Index out of bounds
                }
              } else {
                if (features[81] <= 0.689207f) {
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
      }
    } else {
      if (features[24] <= -0.441602f) {
        if (features[46] <= -0.577190f) {
          return 4;
        } else {
          return 0;
        }
      } else {
        return 0;
      }
    }
  } else {
    if (features[36] <= -0.571101f) {
      return 4;
    } else {
      if (features[85] <= 0.529463f) {
        if (features[47] <= -0.353007f) {
          return 0;
        } else {
          return 4;
        }
      } else {
        if (features[24] <= -0.184214f) {
          if (features[22] <= -0.498407f) {
            return 0;
          } else {
            return 4;
          }
        } else {
          return 0;
        }
      }
    }
  }
} else {
  if (features[3] <= -0.644678f) {
    return 1;
  } else {
    if (features[47] <= -0.302637f) {
      return 0;
    } else {
      return 4;
    }
  }
}
}


int detect_activity(float features[]) {
  int votes[5] = {0};
  int n_trees = 5;
  int i, max_votes = 0, predicted_class = 0;
  
  // Count votes for each class
  votes[tree_0(features)]++;
  votes[tree_1(features)]++;
  votes[tree_2(features)]++;
  votes[tree_3(features)]++;
  votes[tree_4(features)]++;
  
  // Find class with most votes
  for (i = 0; i < 5; i++) {
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

