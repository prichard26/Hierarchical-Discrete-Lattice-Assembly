onmessage = function(e) {
    // console.log('Worker received message:', e.data);
    const { id, type, data } = e.data;
    if (type === "ik3R") {
        const {
            x, y, L0, L1, L2, L3, psi, transitionType
          } = data;
          
        const result = ik3R(x, y, L0, L1, L2, L3, psi, transitionType || 'Default');
        // console.log('Worker sending result:', result);
        postMessage({ id, type: "ik3R", result });
    }
};

function ik3R(x, y, L0, L1, L2, L3, psi, transitionType) {
    let x2, y2;
    if (!transitionType || transitionType === 'None') {
        transitionType = 'Default';
      }
    switch (transitionType) {
        case 'Convex':
          x2 = x - L3 * Math.cos(-psi) - L0;
          y2 = y + L3 * Math.sin(-psi);
          break;
        case 'ConvexSwap':
          x2 = x - L3 * Math.cos(psi) - L0;
          y2 = y + L3 * Math.sin(-psi);
          break;
        case 'ConcaveSwap':
          x2 = x - L3 * Math.sin(psi);
          y2 = y - L3 * Math.cos(-psi) - L0;
          break;
        case 'Default':  // <- Treat this like your old "else"
        default:
          x2 = x - L3 * Math.sin(psi);
          y2 = y - L3 * Math.cos(psi) - L0;
          break;
      }
    
    const dSquared = x2 ** 2 + y2 ** 2;
    const cosTheta2 = Math.max(-1, Math.min(1, (dSquared - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)));
    const theta2 = Math.acos(cosTheta2);
    const theta1 = Math.atan2(y2, x2) - Math.atan2(L2 * Math.sin(theta2), L1 + L2 * Math.cos(theta2));
    const theta3 = psi - (theta1 + theta2);
    return { theta1, theta2, theta3 };
}