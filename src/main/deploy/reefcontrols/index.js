// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import { NT4_Client } from "./NT4.js";

// ***** NETWORKTABLES *****

const toRobotPrefix = "/ReefControls/ToRobot/";
const toDashboardPrefix = "/ReefControls/ToDashboard/";
const selectedLevelTopicName = "SelectedLevel";
const l1TopicName = "Level1";
const l2TopicName = "Level2";
const l3TopicName = "Level3";
const l4TopicName = "Level4";
const algaeTopicName = "Algae";
const coopTopicName = "Coop";
const isElimsTopicName = "IsElims";

const ntClient = new NT4_Client(
  window.location.hostname,
  "ReefControls",
  () => {
    // Topic announce
  },
  () => {
    // Topic unannounce
  },
  (topic, _, value) => {
    // New data
    if (topic.name === toDashboardPrefix + selectedLevelTopicName) {
      selectedLevel = value;
    } else if (topic.name === toDashboardPrefix + l1TopicName) {
      l1State = value;
    } else if (topic.name === toDashboardPrefix + l2TopicName) {
      l2State = value;
    } else if (topic.name === toDashboardPrefix + l3TopicName) {
      l3State = value;
    } else if (topic.name === toDashboardPrefix + l4TopicName) {
      l4State = value;
    } else if (topic.name === toDashboardPrefix + algaeTopicName) {
      algaeState = value;
    } else if (topic.name === toDashboardPrefix + coopTopicName) {
      coopState = value;
    } else if (topic.name === toDashboardPrefix + isElimsTopicName) {
      isElims = value;
    } else {
      return;
    }
    updateUI();
  },
  () => {
    // Connected
  },
  () => {
    // Disconnected
    document.body.style.backgroundColor = "red";
  }
);

// Start NT connection
window.addEventListener("load", () => {
  ntClient.subscribe(
    [
      toDashboardPrefix + selectedLevelTopicName,
      toDashboardPrefix + l1TopicName,
      toDashboardPrefix + l2TopicName,
      toDashboardPrefix + l3TopicName,
      toDashboardPrefix + l4TopicName,
      toDashboardPrefix + algaeTopicName,
      toDashboardPrefix + coopTopicName,
      toDashboardPrefix + isElimsTopicName,
    ],
    false,
    false,
    0.02
  );

  ntClient.publishTopic(toRobotPrefix + selectedLevelTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + l1TopicName, "int");
  ntClient.publishTopic(toRobotPrefix + l2TopicName, "int");
  ntClient.publishTopic(toRobotPrefix + l3TopicName, "int");
  ntClient.publishTopic(toRobotPrefix + l4TopicName, "int");
  ntClient.publishTopic(toRobotPrefix + algaeTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + coopTopicName, "boolean");
  ntClient.connect();
});

// ***** STATE CACHE *****

let selectedLevel = 0; // 0 = L2, 1 = L3, 2 = L4
let l1State = 0; // Count
let l2State = 0; // Bitfield
let l3State = 0; // Bitfield
let l4State = 0; // Bitfield
let algaeState = 0; // Bitfield
let coopState = false; // Boolean
let isElims = false; // Boolean

/** Update the full UI based on the state cache. */
function updateUI() {
  // Update counter highlight
  Array.from(document.getElementsByClassName("counter-area")).forEach(
    (element, index) => {
      if (index > 0 && selectedLevel === index - 1) {
        element.classList.add("active");
      } else {
        element.classList.remove("active");
      }
    }
  );

  // Update background color
  switch (selectedLevel) {
    case 0:
      document.body.style.backgroundColor = "#ccffff";
      break;
    case 1:
      document.body.style.backgroundColor = "#b5e5ff";
      break;
    case 2:
      document.body.style.backgroundColor = "#dfbbfc";
      break;
  }

  // Update level counts
  let rpLevelCount = 0;
  Array.from(document.getElementsByClassName("counter")).forEach(
    (element, index) => {
      if (index === 0) {
        element.innerText = l1State;
        if (l1State >= 7) rpLevelCount++;
      } else {
        let count = 0;
        let levelState = [l2State, l3State, l4State][index - 1];
        for (let i = 0; i < 12; i++) {
          if (((1 << i) & levelState) > 0) {
            count++;
          }
        }
        element.innerText = count === 12 ? "\u2705" : count;
        if (count >= 7) rpLevelCount++;
      }
    }
  );

  // Update coral buttons
  Array.from(document.getElementsByClassName("branch")).forEach(
    (element, index) => {
      let levelState = [l2State, l3State, l4State][selectedLevel];
      if (((1 << index) & levelState) > 0) {
        element.classList.add("active");
      } else {
        element.classList.remove("active");
      }
    }
  );

  // Update algae buttons
  Array.from(document.getElementsByClassName("algae")).forEach(
    (element, index) => {
      if (((1 << index) & algaeState) > 0) {
        element.classList.add("active");
      } else {
        element.classList.remove("active");
      }
    }
  );

  // Update coop button
  let coopDiv = document.getElementsByClassName("coop")[0];
  if (coopState) {
    coopDiv.classList.add("active");
  } else {
    coopDiv.classList.remove("active");
  }

  // Update RP flag
  document.getElementsByClassName("flag")[0].hidden =
    isElims || rpLevelCount < (coopState ? 3 : 4);

  // Update elims state
  if (isElims) {
    document.body.classList.add("elims");
  } else {
    document.body.classList.remove("elims");
  }
}

// ***** BUTTON BINDINGS *****

let isTouch = false;

function bind(element, callback) {
  let activate = (touchEvent) => {
    if (touchEvent) {
      isTouch = true;
    }
    if (isTouch == touchEvent) {
      callback();
    }
  };

  element.addEventListener("touchstart", () => activate(true));
  element.addEventListener("mousedown", () => activate(false));
  element.addEventListener("contextmenu", (event) => {
    event.preventDefault();
    activate(false);
  });
}

let lastMouseEvent = 0;
window.addEventListener("mousemove", () => {
  let now = new Date().getTime();
  if (now - lastMouseEvent < 50) {
    isTouch = false;
  }
  lastMouseEvent = now;
});

window.addEventListener("load", () => {
  // Buttons to change selected level
  Array.from(document.getElementsByClassName("counter-area")).forEach(
    (element, index) => {
      if (index > 0) {
        bind(element, () => {
          ntClient.addSample(toRobotPrefix + selectedLevelTopicName, index - 1);
        });
      }
    }
  );

  // Coral toggle buttons
  Array.from(document.getElementsByClassName("branch")).forEach(
    (element, index) => {
      bind(element, () => {
        switch (selectedLevel) {
          case 0:
            ntClient.addSample(
              toRobotPrefix + l2TopicName,
              l2State ^ (1 << index)
            );
            break;
          case 1:
            ntClient.addSample(
              toRobotPrefix + l3TopicName,
              l3State ^ (1 << index)
            );
            break;
          case 2:
            ntClient.addSample(
              toRobotPrefix + l4TopicName,
              l4State ^ (1 << index)
            );
            break;
        }
      });
    }
  );

  // Algae toggle buttons
  Array.from(document.getElementsByClassName("algae")).forEach(
    (element, index) => {
      bind(element, () => {
        ntClient.addSample(
          toRobotPrefix + algaeTopicName,
          algaeState ^ (1 << index)
        );
      });
    }
  );

  // L1 count controls
  bind(document.getElementsByClassName("subtract")[0], () => {
    if (l1State > 0) {
      ntClient.addSample(toRobotPrefix + l1TopicName, l1State - 1);
    }
  });
  bind(document.getElementsByClassName("add")[0], () => {
    ntClient.addSample(toRobotPrefix + l1TopicName, l1State + 1);
  });

  // Coop button
  bind(document.getElementsByClassName("coop")[0], () => {
    ntClient.addSample(toRobotPrefix + coopTopicName, !coopState);
  });
});

// ***** REEF CANVAS *****

window.addEventListener("load", () => {
  const canvas = document.getElementsByTagName("canvas")[0];
  const context = canvas.getContext("2d");

  let render = () => {
    const devicePixelRatio = window.devicePixelRatio;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    canvas.width = width * devicePixelRatio;
    canvas.height = height * devicePixelRatio;
    context.scale(devicePixelRatio, devicePixelRatio);
    context.clearRect(0, 0, width, height);

    const corners = [
      [width * 0.74, height * 0.9],
      [width * 0.26, height * 0.9],
      [width * 0.03, height * 0.5],
      [width * 0.26, height * 0.1],
      [width * 0.74, height * 0.1],
      [width * 0.97, height * 0.5],
    ];

    context.beginPath();
    corners.forEach((corner) => {
      context.moveTo(width * 0.5, height * 0.5);
      context.lineTo(...corner);
    });
    corners.forEach((corner, index) => {
      if (index == 0) {
        context.moveTo(...corner);
      } else {
        context.lineTo(...corner);
      }
    });
    context.closePath();

    context.strokeStyle = "black";
    context.stroke();
  };

  render();
  window.addEventListener("resize", render);
});
