import { agvStore } from '../store/index.js';


export const navbar = (() => {

    function setup() {
        updateConnectionStatus(agvStore.getState());

        agvStore.on('change', handleChange);
    }

    function updateConnectionStatus(newState) {
        const wifiIcon = document.querySelector('.mdi-wifi');
        if (wifiIcon) {
            wifiIcon.classList.toggle('is-connected', !!newState.isConnected);
            wifiIcon.classList.toggle('is-disconnected', !newState.isConnected);
        }
    }

    function handleChange(newState) {
        updateConnectionStatus(newState);
    }


    return {
        setup,
    };
})();