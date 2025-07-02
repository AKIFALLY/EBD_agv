import { clientStore } from '../store/index.js';


export const navbar = (() => {

    function setup() {
        updateNavbar(clientStore.getState());
        updateConnectionStatus(clientStore.getState());

        clientStore.on('change', handleChange);
    }

    function updateNavbar(newState) {
        const navMachineNumber = document.getElementById('nav-machine-number');
        if (navMachineNumber) {
            navMachineNumber.textContent = 'Machine ' + newState.machineId;
        }
    }

    function updateConnectionStatus(newState) {
        const wifiIcon = document.querySelector('.mdi-wifi');
        if (wifiIcon) {
            wifiIcon.classList.toggle('is-connected', !!newState.isConnected);
            wifiIcon.classList.toggle('is-disconnected', !newState.isConnected);
        }
    }

    function handleChange(newState) {
        updateNavbar(newState);
        updateConnectionStatus(newState);
    }


    return {
        setup,
    };
})();