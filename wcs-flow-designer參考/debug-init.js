const { chromium } = require('playwright');
const path = require('path');

async function debugInitialization() {
  const browser = await chromium.launch({ headless: false });
  const page = await browser.newPage();
  
  const allLogs = [];
  const allErrors = [];
  
  page.on('console', msg => {
    const log = `[${msg.type().toUpperCase()}] ${msg.text()}`;
    console.log(log);
    allLogs.push(log);
  });
  
  page.on('pageerror', error => {
    const errorMsg = `PAGE ERROR: ${error.message}`;
    console.error(errorMsg);
    console.error('Stack:', error.stack);
    allErrors.push(errorMsg);
  });
  
  try {
    const appFile = path.join(__dirname, 'index.html');
    console.log('Loading:', appFile);
    
    await page.goto(`file://${appFile}`);
    await page.waitForTimeout(3000);
    
    // Check step by step what's happening
    const stepByStep = await page.evaluate(() => {
      const results = {};
      
      try {
        results.step1_Rete = typeof window.Rete;
        results.step2_ReteAreaPlugin = typeof window.ReteAreaPlugin;
        results.step3_NodeEditor = typeof window.Rete?.NodeEditor;
        results.step4_ClassicPreset = typeof window.Rete?.ClassicPreset;
        
        // Try to create instances manually
        try {
          const editor = new window.Rete.NodeEditor();
          results.step5_editor_created = 'success';
        } catch (e) {
          results.step5_editor_created = e.message;
        }
        
        try {
          const container = document.getElementById('rete');
          const area = new window.ReteAreaPlugin.AreaPlugin(container);
          results.step6_area_created = 'success';
        } catch (e) {
          results.step6_area_created = e.message;
        }
        
        try {
          const socket = new window.Rete.ClassicPreset.Socket('flow');
          results.step7_socket_created = 'success';
        } catch (e) {
          results.step7_socket_created = e.message;
        }
        
        // Check if WcsFlowDesigner class exists
        results.step8_class_exists = typeof WcsFlowDesigner;
        
        // Try to create WcsFlowDesigner manually
        try {
          const designer = new WcsFlowDesigner();
          results.step9_designer_created = 'success';
          results.step10_designer_properties = Object.keys(designer);
        } catch (e) {
          results.step9_designer_created = e.message;
        }
        
      } catch (e) {
        results.error = e.message;
      }
      
      return results;
    });
    
    console.log('\n=== STEP BY STEP DEBUG ===');
    console.log(JSON.stringify(stepByStep, null, 2));
    
    console.log('\n=== ALL LOGS ===');
    allLogs.forEach(log => console.log(log));
    
    console.log('\n=== ALL ERRORS ===');
    allErrors.forEach(error => console.log(error));
    
    await page.waitForTimeout(2000);
    
  } catch (error) {
    console.error('Debug error:', error);
  } finally {
    await browser.close();
  }
}

debugInitialization();