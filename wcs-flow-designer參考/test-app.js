const { chromium } = require('playwright');
const path = require('path');

async function testApplication() {
  const browser = await chromium.launch({ headless: false, slowMo: 1000 });
  const page = await browser.newPage();
  
  // Listen to console logs and errors
  page.on('console', msg => {
    console.log(`BROWSER [${msg.type()}]:`, msg.text());
  });
  
  page.on('pageerror', error => {
    console.error('PAGE ERROR:', error.message);
  });
  
  try {
    const appFile = path.join(__dirname, 'index.html');
    console.log('Loading application:', appFile);
    
    await page.goto(`file://${appFile}`);
    await page.waitForTimeout(3000); // Wait for scripts to load
    
    console.log('=== TESTING APPLICATION ===');
    
    // Test if the application loaded without errors
    const title = await page.title();
    console.log('Page title:', title);
    
    // Check if main elements are present
    const sidebar = await page.$('.sidebar');
    const editor = await page.$('#rete');
    const toolbar = await page.$('.toolbar');
    
    console.log('Sidebar present:', !!sidebar);
    console.log('Editor present:', !!editor);
    console.log('Toolbar present:', !!toolbar);
    
    // Test clicking a node from palette
    console.log('\n=== TESTING NODE CREATION ===');
    await page.click('[data-node-type="receiving"]');
    await page.waitForTimeout(1000);
    
    // Check if node was created
    const nodeCount = await page.textContent('#nodeCount');
    console.log('Node count after adding receiving node:', nodeCount);
    
    // Try to add another node
    await page.click('[data-node-type="sorting"]');
    await page.waitForTimeout(1000);
    
    const nodeCount2 = await page.textContent('#nodeCount');
    console.log('Node count after adding sorting node:', nodeCount2);
    
    // Test status bar
    const statusBar = await page.textContent('.status-bar');
    console.log('Status bar content:', statusBar);
    
    console.log('\n=== TEST COMPLETED ===');
    
    // Keep browser open for manual inspection
    await page.waitForTimeout(5000);
    
  } catch (error) {
    console.error('Test error:', error);
  } finally {
    await browser.close();
  }
}

testApplication();