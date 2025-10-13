console.log('Token extension loaded');

var elm = document.createElement('script');
elm.innerHTML = `
window.TRACKLESS=${window.TRACKLESS};
window.agvPanelToken="${window.agvPanelToken}";
window.agvPanelTokenX="${window.agvPanelTokenX}";
`;
document.head.appendChild(elm);
