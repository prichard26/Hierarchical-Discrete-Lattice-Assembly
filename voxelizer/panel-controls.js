const controls = document.getElementById('controls');
const togglePanelBtn = document.getElementById('togglePanelBtn');
const openPanelBtn = document.getElementById('openPanelBtn');

togglePanelBtn.addEventListener('click', () => {
  controls.style.display = 'none';
  togglePanelBtn.style.display = 'none';
  openPanelBtn.style.display = 'block';
});

openPanelBtn.addEventListener('click', () => {
  controls.style.display = 'block';
  togglePanelBtn.style.display = 'block';
  openPanelBtn.style.display = 'none';
});



document.getElementById('resetButton').addEventListener('click', () => {
  window.location.reload();
});