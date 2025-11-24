/**
 * This test suite uses Happy DOM as the test environment.
 */
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { ControlTab } from '../src/pages/RobotDashboardPage';

// Only mock the robot connection hook
vi.mock('../src/hooks/useRobotConnection');

describe('ControlTab - Arrow Key Movement', () => {
  let mockSendCommand;

  beforeEach(() => {
    mockSendCommand = vi.fn().mockResolvedValue(true);
    vi.useFakeTimers();
  });

  afterEach(() => {
    vi.clearAllMocks();
    vi.useRealTimers();
  });

  const renderControlTab = () => {
    const videoRef = { current: null };
    return render(
      <ControlTab
        videoRef={videoRef}
        isConnected={true}
        connectionStatus="connected"
        batteryVoltage={12.5}
        wifiStrength={-45}
        robotIP="192.168.1.100"
        sendCommand={mockSendCommand}
        error={null}
      />
    );
  };

  it('should send "forward" command immediately on mouse down', async () => {
    renderControlTab();
    const forwardButton = screen.getByRole('button', { name: /forward/i });
    
    fireEvent.mouseDown(forwardButton);
    
    // Should send immediately
    expect(mockSendCommand).toHaveBeenCalledWith('forward');
  });

  it('should send continuous "forward" commands at 10 Hz (100ms intervals)', async () => {
    renderControlTab();
    const forwardButton = screen.getByRole('button', { name: /forward/i });
    
    fireEvent.mouseDown(forwardButton);
    
    // Initial call
    expect(mockSendCommand).toHaveBeenCalledWith('forward');
    const initialCalls = mockSendCommand.mock.calls.length;
    
    // After 100ms, should have 1 more call
    vi.advanceTimersByTime(100);
    expect(mockSendCommand).toHaveBeenCalledTimes(initialCalls + 1);
    
    // After another 100ms, should have another call
    vi.advanceTimersByTime(100);
    expect(mockSendCommand).toHaveBeenCalledTimes(initialCalls + 2);
    
    // After 1 full second (10 intervals), should have 10 more calls
    mockSendCommand.mockClear();
    vi.advanceTimersByTime(1000);
    expect(mockSendCommand).toHaveBeenCalledTimes(10);
  });

  it('should send "stop" command on mouse up', async () => {
    renderControlTab();
    const forwardButton = screen.getByRole('button', { name: /forward/i });
    
    fireEvent.mouseDown(forwardButton);
    vi.advanceTimersByTime(300);
    
    mockSendCommand.mockClear();
    fireEvent.mouseUp(forwardButton);
    
    expect(mockSendCommand).toHaveBeenCalledWith('stop');
  });

  it('should stop sending commands after mouse up', async () => {
    renderControlTab();
    const forwardButton = screen.getByRole('button', { name: /forward/i });
    
    fireEvent.mouseDown(forwardButton);
    vi.advanceTimersByTime(200);
    
    fireEvent.mouseUp(forwardButton);
    const callsAfterRelease = mockSendCommand.mock.calls.length;
    
    // Advance time - no more movement commands should be sent
    vi.advanceTimersByTime(500);
    
    // Should only have the stop call, no additional movement commands
    expect(mockSendCommand).toHaveBeenCalledTimes(callsAfterRelease);
  });

  it('should send "back" command', async () => {
    renderControlTab();
    const backButton = screen.getByRole('button', { name: /back/i });
    
    fireEvent.mouseDown(backButton);
    expect(mockSendCommand).toHaveBeenCalledWith('back');
  });

  it('should send "left" command', async () => {
    renderControlTab();
    const leftButton = screen.getByRole('button', { name: /left/i });
    
    fireEvent.mouseDown(leftButton);
    expect(mockSendCommand).toHaveBeenCalledWith('left');
  });

  it('should send "right" command', async () => {
    renderControlTab();
    const rightButton = screen.getByRole('button', { name: /right/i });
    
    fireEvent.mouseDown(rightButton);
    expect(mockSendCommand).toHaveBeenCalledWith('right');
  });

  it('should send "stop" on mouse leave', async () => {
    renderControlTab();
    const forwardButton = screen.getByRole('button', { name: /forward/i });
    
    fireEvent.mouseDown(forwardButton);
    vi.advanceTimersByTime(200);
    
    mockSendCommand.mockClear();
    fireEvent.mouseLeave(forwardButton);
    
    expect(mockSendCommand).toHaveBeenCalledWith('stop');
  });
  
});