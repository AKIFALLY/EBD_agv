const { chromium } = require('playwright');
const path = require('path');

async function debugApplication() {
  const browser = await chromium.launch({ headless: false });
  const page = await browser.newPage();
  
  // Capture all console messages and errors
  page.on('console', msg => {
    console.log(`[${msg.type().toUpperCase()}]`, msg.text());
  });
  
  page.on('pageerror', error => {
    console.error('PAGE ERROR:', error.message);
    console.error('Stack:', error.stack);
  });
  
  try {
    const appFile = path.join(__dirname, 'index.html');
    await page.goto(`file://${appFile}`);
    await page.waitForTimeout(2000);
    
    console.log('\n=== DEBUGGING FUNCTIONALITY ===');
    
    // Check if WcsFlowDesigner instance exists
    const hasDesigner = await page.evaluate(() => {
      return typeof window.wcsDesigner !== 'undefined';
    });
    console.log('WcsFlowDesigner instance exists:', hasDesigner);
    
    // Check if Rete objects are available
    const reteInfo = await page.evaluate(() => {
      return {
        Rete: typeof window.Rete,
        ReteAreaPlugin: typeof window.ReteAreaPlugin,
        ReteConnectionPlugin: typeof window.ReteConnectionPlugin,
        NodeEditor: typeof window.Rete?.NodeEditor,
        ClassicPreset: typeof window.Rete?.ClassicPreset
      };
    });
    console.log('Rete objects:', reteInfo);
    
    // Test button click events
    console.log('\n=== TESTING BUTTON CLICKS ===');
    
    // Test node palette clicks
    try {
      console.log('Clicking receiving node...');
      await page.click('[data-node-type="receiving"]');
      await page.waitForTimeout(500);
      
      const nodeCount = await page.textContent('#nodeCount');
      console.log('Node count after click:', nodeCount);
    } catch (error) {
      console.error('Error clicking node:', error.message);
    }
    
    // Test toolbar buttons
    try {
      console.log('Clicking New Flow button...');
      await page.click('.toolbar-button');
      await page.waitForTimeout(500);
    } catch (error) {
      console.error('Error clicking toolbar:', error.message);
    }
    
    // Check event listeners
    const eventListeners = await page.evaluate(() => {
      const receivingNode = document.querySelector('[data-node-type="receiving"]');
      const toolbarButton = document.querySelector('.toolbar-button');
      
      return {
        receivingNodeHasClick: !!receivingNode && receivingNode.onclick !== null,
        toolbarButtonHasClick: !!toolbarButton && toolbarButton.onclick !== null,
        receivingNodeListeners: receivingNode ? getEventListeners(receivingNode) : 'N/A',
        toolbarButtonListeners: toolbarButton ? getEventListeners(toolbarButton) : 'N/A'
      };
    }).catch(() => {
      return { error: 'Cannot access getEventListeners' };
    });
    
    console.log('Event listeners:', eventListeners);
    
    await page.waitForTimeout(3000);
    
  } catch (error) {
    console.error('Debug error:', error);
  } finally {
    await browser.close();
  }
}

debugApplication();