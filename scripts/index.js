const loginElement = document.querySelector('#login-form'); 
const contentElement = document.querySelector("#content-sign-in"); 
const userDetailsElement = document.querySelector('#user-details'); 
const authBarElement = document.querySelector("#authentication-bar"); 
 
// Elements for sensor readings 
const elevationElement = document.getElementById("elevation"); 
const fuelUsedElement = document.getElementById("fuelUsed"); 
const monoxideElement = document.getElementById("monoxide");
const vibrationElement = document.getElementById("vibration");
const fuzzyOutputElement = document.getElementById("fuzzyOutput");
const fuzzyArrowElement = document.getElementById("fuzzyArrow");

// Priority level thresholds
const LOW_THRESHOLD = 60;
const HIGH_THRESHOLD = 120;

// Function to update the gauge display
function updateGauge(value) {
  // Convert the value to a rotation between 0 and 180 degrees
  let rotation = value;
  if (rotation < 0) rotation = 0;
  if (rotation > 180) rotation = 180;
  
  // Update the arrow rotation
  fuzzyArrowElement.style.transform = `rotate(${rotation - 90}deg)`;
  
  // Update the text and class based on the priority level
  let priorityText;
  let priorityClass;
  
  if (value < LOW_THRESHOLD) {
    priorityText = "LOW";
    priorityClass = "priority-low";
  } else if (value < HIGH_THRESHOLD) {
    priorityText = "MEDIUM";
    priorityClass = "priority-medium";
  } else {
    priorityText = "HIGH";
    priorityClass = "priority-high";
  }
  
  fuzzyOutputElement.textContent = `${value.toFixed(1)}° (${priorityText})`;
  fuzzyOutputElement.className = "reading " + priorityClass;
}
 
// MANAGE LOGIN/LOGOUT UI 
const setupUI = (user) => { 
  if (user) { 
    //toggle UI elements 
    loginElement.style.display = 'none'; 
    contentElement.style.display = 'block'; 
    authBarElement.style.display ='block'; 
    userDetailsElement.style.display ='block'; 
    userDetailsElement.innerHTML = user.email; 
 
    // get user UID to get data from database 
    var uid = user.uid; 
    console.log(uid); 
 
    // Database paths (with user UID) 
    var dbPathElevation = 'UsersData/' + uid.toString() + '/elevation'; 
    var dbPathFuelUsed = 'UsersData/' + uid.toString() + '/fuelUsed'; 
    var dbPathMonoxide = 'UsersData/' + uid.toString() + '/monoxideLevel'; 
    var dbPathVibration = 'UsersData/' + uid.toString() + '/vibration';
    var dbPathFuzzyOutput = 'UsersData/' + uid.toString() + '/fuzzyOutput';
 
    // Database references 
    var dbRefElevation = firebase.database().ref().child(dbPathElevation); 
    var dbRefFuelUsed = firebase.database().ref().child(dbPathFuelUsed); 
    var dbRefMonoxide = firebase.database().ref().child(dbPathMonoxide);
    var dbRefVibration = firebase.database().ref().child(dbPathVibration);
    var dbRefFuzzyOutput = firebase.database().ref().child(dbPathFuzzyOutput);
 
    // Update page with new readings 
    dbRefElevation.on('value', snap => { 
      elevationElement.innerText = snap.val().toFixed(2); 
    }); 
 
    dbRefFuelUsed.on('value', snap => { 
      fuelUsedElement.innerText = snap.val().toFixed(2); 
    }); 
 
    dbRefMonoxide.on('value', snap => { 
      monoxideElement.innerText = snap.val().toFixed(2); 
    });
    
    dbRefVibration.on('value', snap => {
      vibrationElement.innerText = snap.val().toFixed(2);
    });
    
    dbRefFuzzyOutput.on('value', snap => {
      const value = snap.val();
      updateGauge(value);
    });
 
  // if user is logged out 
  } else{ 
    // toggle UI elements 
    loginElement.style.display = 'block'; 
    authBarElement.style.display ='none'; 
    userDetailsElement.style.display ='none'; 
    contentElement.style.display = 'none'; 
  } 
}