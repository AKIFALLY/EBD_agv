const { chromium } = require('playwright');
const path = require('path');

async function testConnection() {
  const browser = await chromium.launch({ headless: false, slowMo: 2000 });
  const page = await browser.newPage();
  
  page.on('console', msg => {
    if (!msg.text().includes('parser-blocking')) {
      console.log(`[${msg.type().toUpperCase()}]`, msg.text());
    }
  });
  
  try {
    const appFile = path.join(__dirname, 'index.html');
    await page.goto(`file://${appFile}`);
    await page.waitForTimeout(3000);
    
    console.log('=== CONNECTION TEST ===');
    
    // Create two nodes
    console.log('1. Creating nodes...');
    await page.click('[data-node-type="receiving"]');
    await page.waitForTimeout(1000);
    await page.click('[data-node-type="sorting"]');
    await page.waitForTimeout(1000);
    
    // Check nodes were created
    const nodeCount = await page.textContent('#nodeCount');
    console.log('Nodes created:', nodeCount);
    
    // Get sockets
    const outputSockets = await page.$$('.rete-socket.output');
    const inputSockets = await page.$$('.rete-socket.input');
    
    console.log('Output sockets:', outputSockets.length);
    console.log('Input sockets:', inputSockets.length);
    
    if (outputSockets.length > 0 && inputSockets.length > 0) {
      console.log('\n2. Attempting connection...');
      
      // Try to connect first output to first input
      const output = outputSockets[0];
      const input = inputSockets[0];
      
      // Check socket positions
      const outputBox = await output.boundingBox();
      const inputBox = await input.boundingBox();
      
      console.log('Output socket at:', outputBox);
      console.log('Input socket at:', inputBox);
      
      if (outputBox && inputBox) {
        // Perform connection action
        console.log('Starting drag from output...');
        await page.mouse.move(outputBox.x + outputBox.width/2, outputBox.y + outputBox.height/2);
        await page.mouse.down();
        
        console.log('Dragging to input...');
        await page.mouse.move(inputBox.x + inputBox.width/2, inputBox.y + inputBox.height/2);  
        await page.mouse.up();
        
        await page.waitForTimeout(2000);
        
        // Check result
        const connectionCount = await page.textContent('#connectionCount');
        console.log('Connection count:', connectionCount);
        
        // Check for visual connection
        const connections = await page.$$('svg[id^="connection-"]');
        console.log('Visual connections:', connections.length);
      }
    }
    
    console.log('\n=== CONNECTION TEST COMPLETE ===');
    
    // Keep open for manual inspection
    await page.waitForTimeout(10000);
    
  } catch (error) {
    console.error('Test error:', error.message);
  } finally {
    await browser.close();
  }
}

testConnection();